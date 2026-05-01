Pybricks + VS Code Setup (Mac, Documented)
Audience: Coaches, mentors, and advanced students setting up Pybricks development on macOS.


Target machine: macOS
Repo: /Users/<username>/Your/Path/To/spike_basecode
Python: 3.13.2
VS Code: 1.108.0
Hub firmware: Pybricks already installed and working via WebIDE

This setup:

Uses a real filesystem + git repo

Uses VS Code as the editor

Uses pybricksdev to deploy code to hubs

Avoids WebIDE as the source of truth

0. Preconditions (verify once)

Open Terminal and verify:

python3 --version


Expected:

Python 3.13.2


Verify VS Code command is available:

code --version


If code is not found:

Open VS Code

Command Palette → Shell Command: Install 'code' command in PATH

1. Go to the existing repo
cd /Users/<username>/Your/Path/To/spike_basecode


Confirm this is a git repo:

git status

2. Create a project-local virtual environment

Why:

Isolates Pybricks tooling

Prevents conflicts with system Python

Makes the repo reproducible on every laptop

python3 -m venv .venv


Activate it:

source .venv/bin/activate


You should now see:

(.venv)


at the beginning of your shell prompt.

3. Install Pybricks tooling into the venv

Upgrade pip first (important with Python 3.13):

pip install --upgrade pip


Install required packages:

pip install pybricks pybricksdev


Sanity check:

pybricksdev --help


You should see the CLI help text.

4. Ensure .gitignore is correct

Edit (or create) .gitignore in the repo root:

.venv/
__pycache__/
*.pyc


Optional but common:

.vscode/


You may choose to commit .vscode/launch.json later so all laptops share the same robot configs.

5. Open the repo in VS Code (correctly)

From Terminal (with venv active):

code .


Important:
You must open the folder, not a single .py file — otherwise VS Code will ignore .vscode/launch.json.

6. Select the correct Python interpreter in VS Code

This step is critical and easy to forget.

In VS Code:

Command Palette → Python: Select Interpreter

Choose:

Python 3.13.2 ('.venv': venv)


Verify at bottom-right of VS Code that .venv is active.

7. Create VS Code launch configuration for Pybricks

Create the folder:

mkdir -p .vscode


Create .vscode/launch.json:

{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Run on BR-01",
      "type": "debugpy",
      "request": "launch",
      "module": "pybricksdev",
      "args": [
        "run",
        "ble",
        "--name",
        "BR-01",
        "${workspaceFolder}/main.py"
      ]
    },
    {
      "name": "Run on BR-02",
      "type": "debugpy",
      "request": "launch",
      "module": "pybricksdev",
      "args": [
        "run",
        "ble",
        "--name",
        "BR-02",
        "${workspaceFolder}/main.py"
      ]
    },
    {
      "name": "Run on BR-03",
      "type": "debugpy",
      "request": "launch",
      "module": "pybricksdev",
      "args": [
        "run",
        "ble",
        "--name",
        "BR-03",
        "${workspaceFolder}/main.py"
      ]
    }
  ]
}


Why this matters:

Each config explicitly targets only one hub

Prevents cross-team collisions at workshops

Keeps kids from deploying to the wrong robot

8. Verify hub discovery (optional but recommended)

Turn on one hub and run:

pybricksdev devices ble


You should see output like:

BR-01
BR-02
BR-03


If you don’t:

Confirm hub firmware is Pybricks

Confirm hub name matches exactly (case-sensitive)

9. Test end-to-end from VS Code

Open any mission file (flat layout is fine):

mission1.py


Open Run and Debug panel

Select Run on BR-01

Press F5

Expected behavior:

VS Code console shows pybricksdev output

Hub connects via BLE

Program downloads

Program runs

If motors move: ✅ success.

10. Daily workflow (document-worthy)

Once per session (Terminal):

cd spike_basecode
source .venv/bin/activate
code .


During development:

Turn on your assigned hub

Open your mission file

Select correct robot in Run/Debug

Press F5

Source of truth:

Git repo on disk

VS Code editor

WebIDE only for firmware or emergency recovery

11. What NOT to rely on (important note)

Pybricks Runner extension robot list

It maintains its own state

It may reset after updates

It is not tied to launch.json

Your authoritative configuration is:

.vscode/launch.json

12.  Virtual Environment Note:
This project uses a local Python virtual environment (.venv).
In VS Code, opening a new terminal will automatically activate it.
If you see (.venv) in the terminal prompt, you are ready to run code.
  - Mac: run “Activate venv”
  - Windows: run “Activate venv” (PowerShell)
     -- If PowerShell blocks scripts, run:
     -- Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

13.  Strongly Recommended VS Code Extensions
To get the best editing experience when working with this project in VS Code, you should install the following extensions:

  - Python (Microsoft)
    Adds Python language support in VS Code.

  - Pylance (Microsoft)
    Provides IntelliSense features such as:
     -- autocomplete
     -- method suggestions
     -- jump-to-definition
     -- helpful warnings while typing

These extensions allow VS Code to better understand the Pybricks APIs used in this project. Without them, the editor may show missing autocomplete or confusing red squiggles even when the code is correct.