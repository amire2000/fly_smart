# VS Code offline analyzer task

Add a dedicated VS Code shell task that launches the recorded-flight analyzer
with `uv run fly-smart-analyze`. The analyzer opens its run-folder picker, so
the task does not hard-code a generated output directory.

Validation: parse `.vscode/tasks.json` and verify the command is present.
