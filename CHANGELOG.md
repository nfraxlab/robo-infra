# CHANGELOG


## v0.5.0 (2026-01-05)

### Features

- Auto-discover classes in API extraction script
  ([#14](https://github.com/nfraxlab/robo-infra/pull/14),
  [`172a2f4`](https://github.com/nfraxlab/robo-infra/commit/172a2f4e66d00df44f0c4dfe2e5bdcd1b1bb7d56))

Co-authored-by: nfrax <alixkhatami@gmail.com>


## v0.4.0 (2026-01-05)

### Chores

- Remove required status checks ([#11](https://github.com/nfraxlab/robo-infra/pull/11),
  [`35a1f1e`](https://github.com/nfraxlab/robo-infra/commit/35a1f1e5f187d5c1678e88a703dc4a1ac00a2fba))

### Documentation

- Add quickstart, troubleshooting, security audit; update architecture and performance
  ([#12](https://github.com/nfraxlab/robo-infra/pull/12),
  [`3de816a`](https://github.com/nfraxlab/robo-infra/commit/3de816ada75c866b2af4aa4ab109daf7263bbab7))

### Features

- Add griffe-based API reference extraction system
  ([#13](https://github.com/nfraxlab/robo-infra/pull/13),
  [`118821b`](https://github.com/nfraxlab/robo-infra/commit/118821ba087b67843149afd5f630d387c6e18c5c))


## v0.3.3 (2026-01-02)

### Bug Fixes

- Update messaging to reflect beta status ([#10](https://github.com/nfraxlab/robo-infra/pull/10),
  [`61ae4d3`](https://github.com/nfraxlab/robo-infra/commit/61ae4d3860d3ed895a60a9d203f7e1dc4eb5e44b))

### Continuous Integration

- Add branch protection settings to dismiss stale reviews
  ([#9](https://github.com/nfraxlab/robo-infra/pull/9),
  [`e9b7e83`](https://github.com/nfraxlab/robo-infra/commit/e9b7e8319d53650540ca92a5907c48d5f2cb9525))

Co-authored-by: nfrax <alixkhatami@gmail.com>


## v0.3.2 (2026-01-02)

### Bug Fixes

- Normalize unicode arrows to ascii ([#8](https://github.com/nfraxlab/robo-infra/pull/8),
  [`175ccf5`](https://github.com/nfraxlab/robo-infra/commit/175ccf50e3cb2545656c1331586310970fee53b0))

Co-authored-by: nfrax <alixkhatami@gmail.com>


## v0.3.1 (2025-12-31)

### Bug Fixes

- Sync formatting and docs updates ([#7](https://github.com/nfraxlab/robo-infra/pull/7),
  [`e2f7a76`](https://github.com/nfraxlab/robo-infra/commit/e2f7a7618371943935ca74e18e4af5e9f5becb9d))

Co-authored-by: nfrax <alixkhatami@gmail.com>


## v0.3.0 (2025-12-30)

### Bug Fixes

- Add PR title enforcement workflow ([#5](https://github.com/nfraxlab/robo-infra/pull/5),
  [`6104172`](https://github.com/nfraxlab/robo-infra/commit/610417296fffda9c3b03d3df69d26ffd7c7231e2))

- Semantic-release push tags before publish ([#6](https://github.com/nfraxlab/robo-infra/pull/6),
  [`f061915`](https://github.com/nfraxlab/robo-infra/commit/f061915156968813ee92040c25c03bf7eb7e6376))

### Continuous Integration

- Switch to semantic-release for clean versioning
  ([`4e574c0`](https://github.com/nfraxlab/robo-infra/commit/4e574c0867ac86bc39a5a598fda6dc886edceeb3))

### Documentation

- Update CONTRIBUTING.md with make pr workflow ([#4](https://github.com/nfraxlab/robo-infra/pull/4),
  [`fddfc7e`](https://github.com/nfraxlab/robo-infra/commit/fddfc7e1bcc603d65ecc940a7813b039530a9986))

### Features

- Add robust make pr automation with contributor-safe workflow
  ([#1](https://github.com/nfraxlab/robo-infra/pull/1),
  [`b72e2c4`](https://github.com/nfraxlab/robo-infra/commit/b72e2c4f1e34f6059d1082baa9962a7ec2332e63))

* chore: regenerate poetry.lock after adding semantic-release

* feat: add robust make pr automation with contributor-safe workflow


## v0.2.6 (2025-12-28)

### Bug Fixes

- **ci**: Detect x.y.0 releases and skip auto-bump to create GitHub Release
  ([`048887a`](https://github.com/nfraxlab/robo-infra/commit/048887a6ceb36898a21d241ed6de007f3b760183))

- **ci**: Only release x.y.0 versions, no auto-bump
  ([`5696c92`](https://github.com/nfraxlab/robo-infra/commit/5696c921041946c964a3f7ab014c668ea0d041e1))

Changed the workflow to: - Only publish when version is x.y.0 (deliberate release) - Skip all
  publish steps for non x.y.0 versions - No more auto-bumping patch version on every commit - GitHub
  Release created automatically for x.y.0 versions

### Continuous Integration

- Create GitHub Release for every version
  ([`34f2d1b`](https://github.com/nfraxlab/robo-infra/commit/34f2d1be2af4fb1c8e320f85374acfbdd1aaf3c3))

- Release v0.2.5
  ([`fa65d14`](https://github.com/nfraxlab/robo-infra/commit/fa65d1401e6193d4d0bd8a087267146f36a7b4ef))

- Release v0.2.6
  ([`cbc6817`](https://github.com/nfraxlab/robo-infra/commit/cbc68175ffebced794ca3026238213695533220a))


## v0.2.4 (2025-12-27)

### Continuous Integration

- Add GitHub Release creation to publish workflow
  ([`abe6208`](https://github.com/nfraxlab/robo-infra/commit/abe6208986506c9b21baaa4b61b4fb4adcf9cfdc))

- Only create GitHub Releases for minor/major versions
  ([`5db17da`](https://github.com/nfraxlab/robo-infra/commit/5db17daec9aa2bea3895a1c0c0dffb9c548db953))

- Release v0.2.4
  ([`b27c80a`](https://github.com/nfraxlab/robo-infra/commit/b27c80a8e87a2c10ba57d34afd6cf75b5b3d7b00))


## v0.2.3 (2025-12-27)

### Continuous Integration

- Release v0.2.3
  ([`6c2e7e4`](https://github.com/nfraxlab/robo-infra/commit/6c2e7e42735a5debaa1df2fa1abc3a5625d66795))

### Documentation

- Add testing guidelines for safety systems
  ([`f818a6d`](https://github.com/nfraxlab/robo-infra/commit/f818a6d54cdeead607e64dd458474a1077e2c620))


## v0.2.2 (2025-12-27)

### Continuous Integration

- Release v0.2.2
  ([`b33ac70`](https://github.com/nfraxlab/robo-infra/commit/b33ac70bd313cf0d7a272736146352fd4c518de0))


## v0.2.1 (2025-12-27)

### Bug Fixes

- Update numpy dependency version in pyproject.toml
  ([`5e687fe`](https://github.com/nfraxlab/robo-infra/commit/5e687fe772e8dd55b9aace1430342c5dbd2c058f))

- **ci**: Prevent docs-changelog race condition with publish workflow
  ([`5eb47e5`](https://github.com/nfraxlab/robo-infra/commit/5eb47e5a00e1fa6f6bc8276f52ddf5ba7c3372ec))

### Chores

- Update package versions for ai-infra and svc-infra to 0.1.166 and 0.1.716 respectively
  ([`3725467`](https://github.com/nfraxlab/robo-infra/commit/37254676a91b537cbc25bad7b05781f1cf6d20c0))

### Continuous Integration

- Lower coverage threshold to 72% for optional dependencies
  ([`0559796`](https://github.com/nfraxlab/robo-infra/commit/055979652fd481bae6d525ae52bbd6a66e0ad86e))

The following modules have low coverage because they require optional dependencies not installed in
  CI: - integrations/ai_infra.py (11%) - requires ai-infra - integrations/svc_infra.py (7%) -
  requires svc-infra - vision/*.py (15-41%) - requires opencv - sensors/cameras/*.py (37-51%) -
  requires camera SDKs

Core functionality remains well-tested at >90% coverage.

- Release v0.2.1
  ([`20bcdc9`](https://github.com/nfraxlab/robo-infra/commit/20bcdc9fa2d224ccaee8875b1e1f6a9cccae9721))

### Documentation

- Update changelog [skip ci]
  ([`a7527e0`](https://github.com/nfraxlab/robo-infra/commit/a7527e0bca389a216ae58c128b1d3b69cbb4a086))

- Update changelog [skip ci]
  ([`fec0641`](https://github.com/nfraxlab/robo-infra/commit/fec064190f994678e064fc460581f4c1f00cd8a0))

- Update changelog [skip ci]
  ([`269e307`](https://github.com/nfraxlab/robo-infra/commit/269e307143c413f3c0ef7b58b8af31b048dc81bc))

### Features

- Add real hardware bus implementations for I2C, SPI, and Serial with tests
  ([`73c7cae`](https://github.com/nfraxlab/robo-infra/commit/73c7cae7ffc1bd5531a2ff1bd9baca093490ec9d))

- Enhance observability integration with Prometheus metrics and health checks
  ([`0ac692c`](https://github.com/nfraxlab/robo-infra/commit/0ac692ca82877ec8e574f5ca29482898a69a830c))

- Added observability module for tracking metrics and health checks in robotics controllers. -
  Implemented metrics for command execution, actuator positions, and safety triggers. - Introduced
  health check functions for controllers and actuators to monitor their status. - Created decorators
  for tracking command execution with metrics. - Updated utility functions to provide structured
  logging setup. - Added unit tests for observability features, ensuring metrics recording and
  health checks function correctly. - Included security policy documentation for reporting
  vulnerabilities and best practices. - Refactored existing code to improve type handling and ensure
  compatibility with FastAPI.

- Enhance optional dependencies and installation instructions for ai-infra, svc-infra, and hardware
  support
  ([`b852ff1`](https://github.com/nfraxlab/robo-infra/commit/b852ff1d1786a61dac6f9105ae9a2cd1e1a269fd))

- Update test configurations and improve test coverage with simulation mode
  ([`6540b96`](https://github.com/nfraxlab/robo-infra/commit/6540b9699762f6eefeae5ae68a3722cce93825d2))

### Testing

- Fix CI failures for optional dependencies
  ([`57be178`](https://github.com/nfraxlab/robo-infra/commit/57be17806d9c37a105db241e2bc13af343d82984))

- Add skip markers to resilience tests when svc-infra not installed - Add skip markers to
  observability tests for health checks/metrics - Add skip markers to example tests for
  uvicorn/ai-infra - Relax benchmark targets in CI environment (10x slower) - Fix security test GPIO
  check to expect PrivilegeError

- Skip integration tests when optional deps not installed
  ([`0c173c1`](https://github.com/nfraxlab/robo-infra/commit/0c173c1ce6be0c9f40c25ab809c604af2c756235))

Add pytest.mark.skipif markers to test classes that require: - ai-infra (TestArmWithAITools,
  TestLockWithAITools, TestRoverWithAITools) - fastapi (TestArmWithAPI, TestLockWithAPI,
  TestRoverWithAPI) - both (TestFullE2EFlow classes)

This allows tests to pass in CI where optional dependencies are not installed.


## v0.1.12 (2025-12-19)

### Continuous Integration

- Enhance release process with changelog generation and version tagging
  ([`75e832b`](https://github.com/nfraxlab/robo-infra/commit/75e832b7968cf118c12d38d64c713cfeb6e4bc3f))

- Release v0.1.12
  ([`8ade79a`](https://github.com/nfraxlab/robo-infra/commit/8ade79ab51c2870db64ac5ef234c7a1ede20e782))


## v0.1.11 (2025-12-19)

### Bug Fixes

- Improve error handling when stopping motors in L298N and TB6612 drivers
  ([`18e9b2b`](https://github.com/nfraxlab/robo-infra/commit/18e9b2bcd41e8954bc53e506516dad5b1e6dc1b1))

- Streamline code formatting and improve readability in controllers and drivers
  ([`5759993`](https://github.com/nfraxlab/robo-infra/commit/57599932fc71ce802d7589ec4124dcd885fb249a))

- Update __all__ exports in controllers and improve test comments
  ([`415dbfd`](https://github.com/nfraxlab/robo-infra/commit/415dbfdcc05160616a632c829ab96440e1bf56b2))

### Continuous Integration

- Release v0.1.11
  ([`2acf666`](https://github.com/nfraxlab/robo-infra/commit/2acf666becfe1e8f8a67545dcdd25aecffb79ec8))


## v0.1.10 (2025-12-18)

### Bug Fixes

- Update pre-commit hooks and improve code formatting in stepper and arduino drivers
  ([`38298c8`](https://github.com/nfraxlab/robo-infra/commit/38298c8e8ec71fe9ac6f3b778bd235190d6fe27a))

### Continuous Integration

- Release v0.1.10
  ([`d2c8c2c`](https://github.com/nfraxlab/robo-infra/commit/d2c8c2cea1acd594dc0979abb0c4119910a22942))


## v0.1.9 (2025-12-18)

### Bug Fixes

- Update workflow trigger to run after CI completion
  ([`9584d07`](https://github.com/nfraxlab/robo-infra/commit/9584d0736a2db8889ba460adc9c65df02db33126))

### Continuous Integration

- Release v0.1.9
  ([`4d74d06`](https://github.com/nfraxlab/robo-infra/commit/4d74d06af670298d9c6108524ab2bfc094252639))


## v0.1.8 (2025-12-18)

### Bug Fixes

- Restrict Python version and improve type hinting in router integrations
  ([`44dc313`](https://github.com/nfraxlab/robo-infra/commit/44dc313a942fb4e71406cffd3a41d22d4f30ebcd))

- Updated Python version constraint in pyproject.toml to be less than 4.0. - Enhanced type hinting
  for tags in svc_infra.py by explicitly defining _tags as a list of strings before passing it to
  APIRouter.

- Update repository links and improve documentation clarity
  ([`8b14538`](https://github.com/nfraxlab/robo-infra/commit/8b14538135080e2d841c4f74ca63443a6cd1e0b7))

### Continuous Integration

- Release v0.1.8
  ([`b71edf7`](https://github.com/nfraxlab/robo-infra/commit/b71edf7fed984722f35831ea597799e57b10264a))

### Refactoring

- Enhance Makefile help commands and improve formatting checks
  ([`9ab246e`](https://github.com/nfraxlab/robo-infra/commit/9ab246e8fb7313faae3d33bfe550479eaa136a49))

- Streamline code formatting and improve readability across multiple files
  ([`b3a79c3`](https://github.com/nfraxlab/robo-infra/commit/b3a79c39f359f1af389ced0028fd95b0f1db889e))


## v0.1.7 (2025-12-13)

### Continuous Integration

- Release v0.1.7
  ([`746522f`](https://github.com/nfraxlab/robo-infra/commit/746522f7d66f2fffe94469662722bb82d349cbe0))


## v0.1.6 (2025-12-13)

### Continuous Integration

- Release v0.1.6
  ([`3be0807`](https://github.com/nfraxlab/robo-infra/commit/3be0807709f31a2977aea9a2f10b0cf02e48d7a2))


## v0.1.5 (2025-12-12)

### Continuous Integration

- Release v0.1.5
  ([`6a7392d`](https://github.com/nfraxlab/robo-infra/commit/6a7392d66203a1c0b6bbef5ab9f9fca967133d14))


## v0.1.4 (2025-12-12)

### Continuous Integration

- Release v0.1.4
  ([`25810a7`](https://github.com/nfraxlab/robo-infra/commit/25810a79de31f5dcc79c1c1830a6846b939c28b6))


## v0.1.3 (2025-12-12)

### Continuous Integration

- Release v0.1.3
  ([`d97af2a`](https://github.com/nfraxlab/robo-infra/commit/d97af2a4b20457aacf82ad64ce9f99b8053ac194))

### Documentation

- Add PyPI badge to README
  ([`f72ac11`](https://github.com/nfraxlab/robo-infra/commit/f72ac1151c0ab05b7b6d05a6891ba4f21085e1ee))


## v0.1.2 (2025-12-12)

### Continuous Integration

- Release v0.1.2
  ([`eae5cf8`](https://github.com/nfraxlab/robo-infra/commit/eae5cf8df55e1286aaa9aed79ef50b20931d0406))

- Trigger release
  ([`19a94c0`](https://github.com/nfraxlab/robo-infra/commit/19a94c00456df3fd62805dd08dad9d5b81448452))


## v0.1.1 (2025-12-12)

### Bug Fixes

- Remove poetry.lock from .gitignore for CI
  ([`9bc433e`](https://github.com/nfraxlab/robo-infra/commit/9bc433e4dd8067afa964c0bbe8aa9a4f35237d58))

### Continuous Integration

- Add PyPI publish workflow (OIDC trusted publisher)
  ([`7186250`](https://github.com/nfraxlab/robo-infra/commit/718625052b3939aa51f3c26c492bdd4a8087acb1))

- Auto version bump on push to main - Build and publish via pypa/gh-action-pypi-publish - Update CI
  to support feature branches

- Release v0.1.1
  ([`5bf3b5d`](https://github.com/nfraxlab/robo-infra/commit/5bf3b5d7aaebb1c4f8473839e4de89d3deef1744))

### Features

- Add Makefile for project setup and development commands
  ([`2ecb090`](https://github.com/nfraxlab/robo-infra/commit/2ecb090b83b5ffabe5c4b1cde89952ccf2c72bae))

- Phase 1 - Project Foundation
  ([`193ff48`](https://github.com/nfraxlab/robo-infra/commit/193ff4852542a0e2f4e7da7df3f3d9b6764c70bb))

- pyproject.toml with Poetry config, dependencies, optional groups - Tool configs: ruff.toml,
  mypy.ini, pytest.ini, .pre-commit-config.yaml - Core types: Limits, Position, Vector3, Direction,
  Unit, Reading - Exception hierarchy: RoboInfraError and specialized exceptions - CLI entrypoint
  (robo-infra) - Module structure: actuators, sensors, drivers, controllers, motion, platforms,
  safety, integrations, utils - GitHub Actions CI workflow - README with architecture overview - 40
  passing unit tests
