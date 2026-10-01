"""Allow ``python -m eilik``; see :mod:`eilik.cli`."""

import sys

from .cli import main

sys.exit(main())
