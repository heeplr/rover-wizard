"""Entry point of the PyInstaller bundle (packaging/rover-wizard.spec).

PyInstaller runs its entry script as a plain top-level script, without a package around it, so rover_wizard's own
__main__.py (relative imports) can't be that script. This one imports the package absolutely and runs the same
main() as the `rover-wizard` command and `python -m rover_wizard`.
"""
import sys

from rover_wizard.__main__ import main

if __name__ == "__main__":
    sys.exit(main())
