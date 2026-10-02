# Interactive Stop exit status

An interactive operator Stop is an intentional unsuccessful attempt, not a
process error. The runner still writes the attempt video, telemetry CSV, plot,
and summary, then closes normally. The CLI therefore does not assert success
for `--interactive` headless sessions; non-interactive headless runs retain
the failure assertion for automation and CI.
