## Test run - build b66b962-dirty (Aug 19 2026 15:49:33)

Note on version: the version committed immediately after the one this test was
done on is the version of the software this particular suite was run on.

Firmware: `b66b962-dirty` | ESP-IDF 6.0.0

| # | Test | Result | Note |
|---|------|--------|------|
| 0 | Self tests (pure math) | PASS | automatic |
| 1 | IMU ctor: zeroed + ranges set | ---- |  |
| 2 | IMU ctor: range saturation | ---- |  |
| 3 | IMU calibrate(): at rest | ---- |  |
| 4 | IMU update(): six-face | ---- |  |
| 5 | Servo::move_to_angle sweep | PASS | Servo #3 passes, POST write consolidation |
| 6 | Servo::move_to_pwm (scoped) | SKIP | implied by move_to_angle |
| 7 | SpotServo #1: limit saturation | PASS | left knee bolt touching, but fine otherwi |
| 8 | SpotServo #2: protractor angles | PASS | ALL WORKS!!! |

**4 passed, 0 failed, 1 skipped, 4 not run**

## Test run - build b66b962-dirty (Aug 19 2026 15:49:33)

Note on version: the version committed immediately after the one this test was
done on is the version of the software this particular suite was run on.

Firmware: `b66b962-dirty` | ESP-IDF 6.0.0

| # | Test | Result | Note |
|---|------|--------|------|
| 0 | Self tests (pure math) | PASS | automatic |
| 1 | IMU ctor: zeroed + ranges set | ---- |  |
| 2 | IMU ctor: range saturation | ---- |  |
| 3 | IMU calibrate(): at rest | ---- |  |
| 4 | IMU update(): six-face | ---- |  |
| 5 | Servo::move_to_angle sweep | PASS | Servo #3 passes, POST write consolidation |
| 6 | Servo::move_to_pwm (scoped) | SKIP | implied by move_to_angle |
| 7 | SpotServo #1: limit saturation | PASS | left knee bolt touching, but fine otherwi |
| 8 | SpotServo #2: protractor angles | ---- |  |

Note on limit saturation, the front left knee joint's servo horn central bolt head
 touches the underside of the body cover, but its chill because it isn't too tight.
Ideally replace the bolt but unncessary.

Additionally, note that these limit tests were performed WITHOUT the top covers on the bot.

**3 passed, 0 failed, 1 skipped, 5 not run**

## Test run - build b66b962-dirty (Aug 19 2026 15:49:33)

Firmware: `b66b962-dirty` | ESP-IDF 6.0.0

| # | Test | Result | Note |
|---|------|--------|------|
| 0 | Self tests (pure math) | PASS | automatic |
| 1 | IMU ctor: zeroed + ranges set | PASS |  |
| 2 | IMU ctor: range saturation | PASS | All pass with 100%! |
| 3 | IMU calibrate(): at rest | PASS | not exactly zero for 4/6 |
| 4 | IMU update(): six-face | PASS | 6 faces pace w almost 100% in right direction |
| 5 | Servo::move_to_angle sweep | PASS | tested on servo 3, passed w flying colors! |
| 6 | Servo::move_to_pwm (scoped) | SKIP | implied by move_to_angle |
| 7 | SpotServo #1: limit saturation | SKIP |  |
| 8 | SpotServo #2: protractor angles | SKIP |  |

**6 passed, 0 failed, 3 skipped, 0 not run**

## Test run - build d063c2a-dirty (Aug 12 2026 23:18:59)

Firmware: `d063c2a-dirty` | ESP-IDF 6.0.0

| # | Test | Result | Note |
|---|------|--------|------|
| 0 | Self tests (pure math) | PASS | automatic |
| 1 | IMU ctor: zeroed + ranges set | PASS |  |
| 2 | IMU ctor: range saturation | PASS | All pass with 100%! |
| 3 | IMU calibrate(): at rest | PASS | not exactly zero for 4/6 |
| 4 | IMU update(): six-face | PASS | 6 faces pace w almost 100% in right direction |
| 5 | Servo::move_to_angle sweep | SKIP |  |
| 6 | Servo::move_to_pwm (scoped) | ---- |  |
| 7 | SpotServo #1: limit saturation | SKIP | Shoulder servo unable to reach 180 degrees. START HERE |
| 8 | SpotServo #2: protractor angles | ---- |  |

**5 passed, 0 failed, 2 skipped, 2 not run**

## Test run - build d063c2a-dirty (Aug 12 2026 23:18:59)

Firmware: `d063c2a-dirty` | ESP-IDF 6.0.0

| # | Test | Result | Note |
|---|------|--------|------|
| 0 | Self tests (pure math) | PASS | automatic |
| 1 | IMU ctor: zeroed + ranges set | PASS |  |
| 2 | IMU ctor: range saturation | PASS | All pass with 100%! |
| 3 | IMU calibrate(): at rest | PASS | not exactly zero for 4/6 |
| 4 | IMU update(): six-face | PASS | 6 faces pace w almost 100% in right direction |
| 5 | Servo::move_to_angle sweep | SKIP |  |
| 6 | Servo::move_to_pwm (scoped) | ---- |  |
| 7 | SpotServo #1: limit saturation | FAIL | Hip and shoulder servos failed to move to their second bound! |
| 8 | SpotServo #2: protractor angles | ---- |  |

**5 passed, 1 failed, 1 skipped, 2 not run**

