# Application functional verification — 2026-09-24

Tested with the actual local HTTP servers and headless Google Chrome on macOS.
This is application functionality testing, not proof of the experimental
Instructor model or Windows/MSVC compatibility.

## Issue found and fixed

Loading a saved altitude result at 110% power restored the number input as
110.00000000000001. HTML maximum validation then prevented the next Calculate
submission. The UI now removes floating-point roundoff when restoring throttle.
The physics input remains 1.1. A regression assertion was added to
`scripts/verify_altitude_browser.py`.

The pre-existing EM server also had older code loaded after the cleanup. Its
source-change guard correctly blocked calculations; the server was restarted.
Restart a running calculator after changing equation files.

## Results

Detailed final reports are recorded in `app-functional-macos.json`.
The altitude application passed real calculation, contours, point inspection,
energy guide and CSV, all five exports, 3D surface/PNG, reload/edit/recalculate,
mobile layout, cancellation and a fresh calculation after cancellation. No
JavaScript page errors were observed. The EM application passed a separate
RB/SB comparison (873/1,061 valid points), a J6K1 RB chart (462 valid points),
both point inspectors, all five exports, saved-result restoration, mobile
layout, cancellation and another calculation afterward. EM also had no
JavaScript page errors.

A focused reload regression confirmed that the restored input is exactly
110%, passes HTML validity, and still submits physics throttle 1.1.

Screenshots and the full test driver are retained outside the project in the
sibling archive's `functional-check` folder. Generated calculation outputs are
excluded from the transfer ZIP. Windows setup still needs to be run on the PC;
these Mac checks do not certify Windows compilation or performance.
