extends Node3D

# Linux shared-memory-backed file. Godot writes; Python maps and reads it.
const SHM_PATH := "/dev/shm/fly_smart_fpv.rgb"
const WIDTH := 640
const HEIGHT := 360
const FPS := 15.0
const HEADER_BYTES := 32
const FRAME_BYTES := WIDTH * HEIGHT * 3  # tightly packed RGB8
const POSE_PORT := 9100

var _drone: Node3D
var _spinning_body: Node3D
var _fpv_viewport: SubViewport
var _fpv_camera: Camera3D
var _shm: FileAccess
var _pose_socket := PacketPeerUDP.new()
var _active_slot := 0
var _sequence := 0
var _time := 0.0
var _latest_pose: Dictionary = {}


func _ready() -> void:
	_build_world()
	_build_drone()
	_build_spinning_body()
	_build_cameras()
	var bind_error := _pose_socket.bind(POSE_PORT, "127.0.0.1")
	if bind_error != OK:
		push_error("Cannot listen for PyBullet poses on UDP %d: %s" % [POSE_PORT, bind_error])
	else:
		print("Waiting for PyBullet poses on UDP 127.0.0.1:", POSE_PORT)
	_open_shared_memory()
	if _shm != null:
		_capture_loop()
		print("FPV shared memory ready: ", SHM_PATH)


func _process(delta: float) -> void:
	_receive_latest_pose()
	_time += delta
	# PyBullet is authoritative. Keep the demo motion only when no sender is
	# connected, so the Godot project can still be previewed by itself.
	if _latest_pose.is_empty():
		_drone.position = Vector3(sin(_time * 0.35) * 2.5, 2.7 + sin(_time) * 0.25, 4.0 - _time * 0.45)
		_drone.rotation = Vector3(sin(_time * 0.7) * 0.08, sin(_time * 0.25) * 0.12, sin(_time * 0.9) * 0.1)
	_fpv_camera.global_transform = _drone.global_transform * Transform3D(Basis.IDENTITY, Vector3(0.0, 0.05, -0.35))
	if _drone.position.z < -18.0:
		_time = 0.0


func _build_world() -> void:
	var env := WorldEnvironment.new()
	var settings := Environment.new()
	settings.background_mode = Environment.BG_COLOR
	settings.background_color = Color(0.50, 0.73, 0.93)
	settings.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	settings.ambient_light_color = Color(0.75, 0.82, 0.9)
	env.environment = settings
	add_child(env)

	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, -25, 0)
	sun.light_energy = 1.5
	add_child(sun)

	_add_box(self, Vector3(0, -0.15, -8), Vector3(80, 0.3, 80), Color(0.34, 0.53, 0.29))
	_add_box(self, Vector3(0, 0.02, -9), Vector3(3.2, 0.05, 42), Color(0.22, 0.24, 0.26))
	for z in range(-28, 12, 5):
		_add_box(self, Vector3(0, 0.06, z), Vector3(0.08, 0.01, 2.0), Color(0.95, 0.86, 0.46))
	for i in range(9):
		var z := 7.0 - float(i) * 5.0
		var height := 1.2 + float(i % 3) * 0.55
		_add_box(self, Vector3(-6, height * 0.5, z), Vector3(2.1, height, 2.1), Color(0.74, 0.36, 0.21))
		_add_box(self, Vector3(6, height * 0.5, z - 2), Vector3(2.1, height, 2.1), Color(0.72, 0.64, 0.42))


func _build_drone() -> void:
	_drone = Node3D.new()
	_drone.name = "Drone"
	add_child(_drone)
	_add_box(_drone, Vector3.ZERO, Vector3(0.42, 0.16, 0.55), Color(0.08, 0.10, 0.13))
	_add_box(_drone, Vector3.ZERO, Vector3(0.9, 0.07, 0.08), Color(0.13, 0.15, 0.18))
	for x in [-0.42, 0.42]:
		_add_box(_drone, Vector3(x, 0.02, 0), Vector3(0.22, 0.05, 0.22), Color(0.94, 0.40, 0.12))


func _build_spinning_body() -> void:
	_spinning_body = Node3D.new()
	_spinning_body.name = "PyBulletBody"
	_spinning_body.position = Vector3(0, 1.8, -10)
	add_child(_spinning_body)
	_add_box(_spinning_body, Vector3.ZERO, Vector3(1.3, 0.6, 0.8), Color(0.92, 0.23, 0.16))
	# The offset colored panels make attitude changes obvious in the FPV image.
	_add_box(_spinning_body, Vector3(0, 0.31, 0), Vector3(1.1, 0.03, 0.5), Color(0.1, 0.75, 0.98))
	_add_box(_spinning_body, Vector3(0.25, 0, 0.41), Vector3(0.3, 0.4, 0.03), Color(0.96, 0.9, 0.2))


func _receive_latest_pose() -> void:
	# Drain the socket: rendering should use the newest physics sample.
	while _pose_socket.get_available_packet_count() > 0:
		var packet := _pose_socket.get_packet().get_string_from_utf8()
		var value: Variant = JSON.parse_string(packet)
		if not value is Dictionary:
			continue
		_latest_pose = value
		_apply_pose(value.get("drone"), _drone)
		_apply_pose(value.get("target"), _spinning_body)


func _apply_pose(raw_pose: Variant, node: Node3D) -> void:
	if not raw_pose is Dictionary:
		return
	var position_b: Variant = raw_pose.get("p")
	var attitude_b: Variant = raw_pose.get("q")
	if not position_b is Array or not attitude_b is Array:
		return
	if position_b.size() != 3 or attitude_b.size() != 4:
		return
	# PyBullet: X right, Y forward, Z up; Godot: X right, -Z forward, Y up.
	var position_g := Vector3(float(position_b[0]), float(position_b[2]), -float(position_b[1]))
	var rotation_g := Quaternion(
		float(attitude_b[0]), float(attitude_b[2]),
		-float(attitude_b[1]), float(attitude_b[3])
	).normalized()
	node.global_transform = Transform3D(Basis(rotation_g), position_g)


func _build_cameras() -> void:
	var overview := Camera3D.new()
	overview.name = "OverviewCamera"
	add_child(overview)
	overview.position = Vector3(12, 10, 16)
	overview.look_at(Vector3(0, 0, -9))
	overview.current = true

	_fpv_viewport = SubViewport.new()
	_fpv_viewport.name = "FPVViewport"
	_fpv_viewport.size = Vector2i(WIDTH, HEIGHT)
	_fpv_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_fpv_viewport.world_3d = get_viewport().world_3d
	add_child(_fpv_viewport)

	_fpv_camera = Camera3D.new()
	_fpv_camera.name = "FPVCamera"
	_fpv_camera.projection = Camera3D.PROJECTION_PERSPECTIVE
	_fpv_camera.fov = 75.0  # vertical FOV when KEEP_HEIGHT is selected
	_fpv_camera.near = 0.05
	_fpv_camera.far = 500.0
	_fpv_viewport.add_child(_fpv_camera)
	_fpv_camera.current = true

	var overlay := CanvasLayer.new()
	add_child(overlay)
	var preview := TextureRect.new()
	preview.name = "FPVPreview"
	preview.position = Vector2(10, 10)
	preview.size = Vector2(480, 270)
	preview.texture = _fpv_viewport.get_texture()
	preview.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	overlay.add_child(preview)


func _add_box(parent: Node, pos: Vector3, size: Vector3, color: Color) -> void:
	var mesh := BoxMesh.new()
	mesh.size = size
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	mesh.material = material
	var instance := MeshInstance3D.new()
	instance.mesh = mesh
	instance.position = pos
	parent.add_child(instance)


func _open_shared_memory() -> void:
	_shm = FileAccess.open(SHM_PATH, FileAccess.WRITE_READ)
	if _shm == null:
		push_error("Cannot open %s: %s" % [SHM_PATH, FileAccess.get_open_error()])
		return
	# Header: magic, width, height, channels, active_slot, sequence, frame_bytes, reserved.
	_shm.big_endian = false
	_shm.store_buffer("GFPV".to_ascii_buffer())
	_shm.store_32(WIDTH)
	_shm.store_32(HEIGHT)
	_shm.store_32(3)
	_shm.store_32(0)
	_shm.store_32(0)
	_shm.store_32(FRAME_BYTES)
	_shm.store_32(0)
	_shm.seek(HEADER_BYTES + 2 * FRAME_BYTES - 1)
	_shm.store_8(0)
	_shm.flush()


func _capture_loop() -> void:
	while is_inside_tree() and _shm != null:
		await RenderingServer.frame_post_draw
		var image := _fpv_viewport.get_texture().get_image()
		if not image.is_empty():
			image.convert(Image.FORMAT_RGB8)
			var pixels := image.get_data()
			if pixels.size() == FRAME_BYTES:
				_write_frame(pixels)
		await get_tree().create_timer(1.0 / FPS).timeout


func _write_frame(pixels: PackedByteArray) -> void:
	var next_slot := 1 - _active_slot
	# Odd sequence means a write is underway. Python retries until it sees
	# the same even sequence before and after copying the chosen slot.
	_shm.seek(20)
	_shm.store_32(_sequence + 1)
	_shm.flush()
	_shm.seek(HEADER_BYTES + next_slot * FRAME_BYTES)
	_shm.store_buffer(pixels)
	_shm.flush()
	_shm.seek(16)
	_shm.store_32(next_slot)
	_shm.store_32(_sequence + 2)
	_shm.flush()
	_active_slot = next_slot
	_sequence += 2
