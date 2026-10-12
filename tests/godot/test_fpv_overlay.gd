extends SceneTree

# Exercise the actual scene's source-pixel to preview-pixel transform.
func _initialize() -> void:
	call_deferred("_run")

func _run() -> void:
	var scene := Node3D.new()
	root.add_child(scene)
	# Attach after entry so startup cannot bind UDP or replace shared memory.
	scene.set_script(load("res://scripts/main.gd"))
	scene.set_process(false)
	scene._build_cameras()
	await process_frame
	scene._update_takeoff_altitude({"takeoff_altitude_m": 15})
	if scene._takeoff_altitude.get_selected_metadata() != 15.0:
		push_error("Configured takeoff altitude was not preserved")
		quit(1)
		return
	scene._update_target_distance({"distance_m": 150})
	var receiver := PacketPeerUDP.new()
	if receiver.bind(0, "127.0.0.1") != OK:
		quit(1)
		return
	scene._control_socket.connect_to_host("127.0.0.1", receiver.get_local_port())
	for altitude in [20, 30, 50]:
		scene._update_takeoff_altitude({"takeoff_altitude_m": altitude})
		scene._send_control("reset")
		await process_frame
		if receiver.get_available_packet_count() == 0:
			push_error("Reset selection packet missing")
			quit(1)
			return
		var packet = JSON.parse_string(receiver.get_packet().get_string_from_utf8())
		if packet.get("takeoff_altitude_m") != altitude or packet.get("target_distance_m") != 150 or packet.get("action") != "reset":
			push_error("Reset selection packet mismatch")
			quit(1)
			return
	receiver.close()
	for dimensions in [Vector2i(640, 480), Vector2i(640, 360), Vector2i(1280, 960)]:
		scene.WIDTH = dimensions.x
		scene.HEIGHT = dimensions.y
		for bbox in [[100, 300, 40, 20], [318, 268, 4, 4], [318, 268, 3, 3]]:
			scene._update_fpv_overlay({"bbox": bbox})
			var scale := Vector2(480, 270) / Vector2(dimensions)
			var expected_position := Vector2(10, 10) + Vector2(bbox[0], bbox[1]) * scale
			var expected_size := Vector2(bbox[2], bbox[3]) * scale
			if not scene._bbox_panel.position.is_equal_approx(expected_position) or not scene._bbox_panel.size.is_equal_approx(expected_size):
				push_error("BBox projection mismatch: %s/%s expected %s/%s" % [scene._bbox_panel.position, scene._bbox_panel.size, expected_position, expected_size])
				quit(1)
				return
	scene._update_fpv_overlay({"bbox": null})
	if scene._bbox_panel.visible:
		push_error("Missing detection must hide the bbox")
		quit(1)
		return
	print("FPV bbox projection and reset selections passed")
	quit(0)
