# Changelog

## [7.4.0](https://github.com/pockerman/bitrl/compare/v7.3.0...v7.4.0) (2026-11-23)


### Features

* evolution strategies ([#229](https://github.com/pockerman/bitrl/issues/229)) ([14d39bf](https://github.com/pockerman/bitrl/commit/14d39bf3b11522f3118c5f65a003a3ed37250964))


### Bug Fixes

* **multi-agent:** repair IndependentQLearning after a bad merge ([#228](https://github.com/pockerman/bitrl/issues/228)) ([7be57f4](https://github.com/pockerman/bitrl/commit/7be57f47ec423aff62a88f805b5a30d47bbb77d7))
* **multi-agent:** repair IndependentQLearning after a bad merge ([#231](https://github.com/pockerman/bitrl/issues/231)) ([921f9a0](https://github.com/pockerman/bitrl/commit/921f9a0997731efb44e2a698ea1369934eb07a31))

## [7.3.0](https://github.com/pockerman/bitrl/compare/v7.2.0...v7.3.0) (2026-11-23)


### Features

* distributed ppo ([#224](https://github.com/pockerman/bitrl/issues/224)) ([13cb054](https://github.com/pockerman/bitrl/commit/13cb054390fb78156231a4cc9fc5fc0076bff7e0))
* parallel ppo ([#223](https://github.com/pockerman/bitrl/issues/223)) ([9687287](https://github.com/pockerman/bitrl/commit/96872879175314734608374704270cfa776856f9))

## [7.2.0](https://github.com/pockerman/bitrl/compare/v7.1.0...v7.2.0) (2026-11-23)


### Features

* distributed multi agent ([#225](https://github.com/pockerman/bitrl/issues/225)) ([dbb8d0c](https://github.com/pockerman/bitrl/commit/dbb8d0c3328c0c0053d38256aaaeec14dd3fa23d))
* multi agent ([#220](https://github.com/pockerman/bitrl/issues/220)) ([9fa4a13](https://github.com/pockerman/bitrl/commit/9fa4a13eb9168d238d8f652859f1f6debcba4458))

## [7.1.0](https://github.com/pockerman/bitrl/compare/v7.0.0...v7.1.0) (2026-11-22)


### Features

* distributed multi agent ([#221](https://github.com/pockerman/bitrl/issues/221)) ([ee0af93](https://github.com/pockerman/bitrl/commit/ee0af932821be679156b13f4bcf26b3903f6ea96))

## [7.0.0](https://github.com/pockerman/bitrl/compare/v6.0.1...v7.0.0) (2026-11-22)


### ⚠ BREAKING CHANGES

* bitrl needs bitsim 5, the first bitsim that logs with spdlog, and neither links Boost.Log nor defines BOOST_LOG_DYN_LINK any more. Code that logged with BOOST_LOG_TRIVIAL through bitrl must link Boost.Log itself or move to bitrl/base/bitrl_log.h.
* building needs bitsim 3.0.0 or later.

### Features

* add envs ([#181](https://github.com/pockerman/bitrl/issues/181)) ([0acbe78](https://github.com/pockerman/bitrl/commit/0acbe7844553daed0ccc6fda9d3dc429c9fbe7ae))
* **algorithms:** ppo cartpole ([#209](https://github.com/pockerman/bitrl/issues/209)) ([cc6de7b](https://github.com/pockerman/bitrl/commit/cc6de7baa8e0928961b7365c08174b154d9a9e93))
* **examples:** export and log experiments ([#206](https://github.com/pockerman/bitrl/issues/206)) ([27ed5b5](https://github.com/pockerman/bitrl/commit/27ed5b5e338b0ae11b619fdaac00ea4d301ca72f))
* **examples:** pytorch policy network ([#204](https://github.com/pockerman/bitrl/issues/204)) ([6e6e8e8](https://github.com/pockerman/bitrl/commit/6e6e8e8d2a1a1498121aaf431d75f8ac77b46e6c))
* log with bitsim's spdlog logger instead of Boost.Log ([#211](https://github.com/pockerman/bitrl/issues/211)) ([d717718](https://github.com/pockerman/bitrl/commit/d7177184bc2ed6927e359939ceb1df3cc390ba54))
* mcts line following ([#196](https://github.com/pockerman/bitrl/issues/196)) ([e92a059](https://github.com/pockerman/bitrl/commit/e92a059485c611778c54b8563300134e3b5afaea))
* multi agent ([#218](https://github.com/pockerman/bitrl/issues/218)) ([2e4257a](https://github.com/pockerman/bitrl/commit/2e4257aff89e0533917afe377036d5abcc3e690c))
* **multi-agent:** multi-agent environments, trainer and independent … ([#214](https://github.com/pockerman/bitrl/issues/214)) ([25bc986](https://github.com/pockerman/bitrl/commit/25bc9868a43ef89daadb3d4130d56cb30816e07e))


### Bug Fixes

* **algorithms:** SARSA takes the next action from the next state ([#199](https://github.com/pockerman/bitrl/issues/199)) ([ce2d590](https://github.com/pockerman/bitrl/commit/ce2d5905593ed1fec410f853f11806b8d6912c8b))
* expected sarsa ([#198](https://github.com/pockerman/bitrl/issues/198)) ([4647e91](https://github.com/pockerman/bitrl/commit/4647e9182bff2e6c7750436cb59098fa1026681d))
* fix compiler warnings ([#190](https://github.com/pockerman/bitrl/issues/190)) ([6ec6ece](https://github.com/pockerman/bitrl/commit/6ec6eced8a8c239b3138e254ff9d533a02b013f3))
* fix multi agent bug ([#216](https://github.com/pockerman/bitrl/issues/216)) ([3d3cb71](https://github.com/pockerman/bitrl/commit/3d3cb719b562c27b22cd332261e39a50b87da8af))
* port the unused headers that were left on the cubeai API ([#195](https://github.com/pockerman/bitrl/issues/195)) ([39e942f](https://github.com/pockerman/bitrl/commit/39e942f10e495f199b1d07377294de9c2d34beac))
* rest json content type ([#208](https://github.com/pockerman/bitrl/issues/208)) ([3c0e1ff](https://github.com/pockerman/bitrl/commit/3c0e1ff58564c0273fef1df2bd749a708596cb17))


### Refactoring

* **examples:** put the examples in numbered sections, and make the README about reinforcement learning ([#203](https://github.com/pockerman/bitrl/issues/203)) ([e3fc644](https://github.com/pockerman/bitrl/commit/e3fc644e2bfb02d2effd2b7e4c13b36b94b9e845))


### Documentation

* fix the pages of examples 8, 13 and 14 ([#182](https://github.com/pockerman/bitrl/issues/182)) ([82df9e6](https://github.com/pockerman/bitrl/commit/82df9e6db4b2e333973436930329b1f6ddece1e8))
* link the renamed docs site ([#213](https://github.com/pockerman/bitrl/issues/213)) ([eb6a08f](https://github.com/pockerman/bitrl/commit/eb6a08f10796ce42c3a15176c4d308bce6c22c5b))
* rewrite the page of example 14 ([#184](https://github.com/pockerman/bitrl/issues/184)) ([12a0fe5](https://github.com/pockerman/bitrl/commit/12a0fe5b0a3b49fa5dcf2ca2b3fbe693e0b8683c))
* show the Monte Carlo tree search of the line following example in the README ([#201](https://github.com/pockerman/bitrl/issues/201)) ([e9c461f](https://github.com/pockerman/bitrl/commit/e9c461f3b8d56146ff7e23f6d1c124c4aebdeee9))


### Build

* require bitsim 3 ([#183](https://github.com/pockerman/bitrl/issues/183)) ([64bb1cd](https://github.com/pockerman/bitrl/commit/64bb1cd6e1d305c786d5f686b15dc092646bb633))

## [6.0.1](https://github.com/pockerman/bitrl/compare/v6.0.0...v6.0.1) (2026-11-22)


### Bug Fixes

* fix multi agent bug ([#216](https://github.com/pockerman/bitrl/issues/216)) ([3d3cb71](https://github.com/pockerman/bitrl/commit/3d3cb719b562c27b22cd332261e39a50b87da8af))

## [6.0.0](https://github.com/pockerman/bitrl/compare/v5.3.0...v6.0.0) (2026-11-21)


### ⚠ BREAKING CHANGES

* bitrl needs bitsim 5, the first bitsim that logs with spdlog, and neither links Boost.Log nor defines BOOST_LOG_DYN_LINK any more. Code that logged with BOOST_LOG_TRIVIAL through bitrl must link Boost.Log itself or move to bitrl/base/bitrl_log.h.

### Features

* log with bitsim's spdlog logger instead of Boost.Log ([#211](https://github.com/pockerman/bitrl/issues/211)) ([d717718](https://github.com/pockerman/bitrl/commit/d7177184bc2ed6927e359939ceb1df3cc390ba54))


### Documentation

* link the renamed docs site ([#213](https://github.com/pockerman/bitrl/issues/213)) ([eb6a08f](https://github.com/pockerman/bitrl/commit/eb6a08f10796ce42c3a15176c4d308bce6c22c5b))

## [5.3.0](https://github.com/pockerman/bitrl/compare/v5.2.0...v5.3.0) (2026-11-21)


### Features

* **algorithms:** ppo cartpole ([#209](https://github.com/pockerman/bitrl/issues/209)) ([cc6de7b](https://github.com/pockerman/bitrl/commit/cc6de7baa8e0928961b7365c08174b154d9a9e93))

## [5.2.0](https://github.com/pockerman/bitrl/compare/v5.1.2...v5.2.0) (2026-11-21)


### Features

* **examples:** export and log experiments ([#206](https://github.com/pockerman/bitrl/issues/206)) ([27ed5b5](https://github.com/pockerman/bitrl/commit/27ed5b5e338b0ae11b619fdaac00ea4d301ca72f))

## [5.1.2](https://github.com/pockerman/bitrl/compare/v5.1.1...v5.1.2) (2026-11-21)


### Refactoring

* **examples:** put the examples in numbered sections, and make the README about reinforcement learning ([#203](https://github.com/pockerman/bitrl/issues/203)) ([e3fc644](https://github.com/pockerman/bitrl/commit/e3fc644e2bfb02d2effd2b7e4c13b36b94b9e845))


### Documentation

* show the Monte Carlo tree search of the line following example in the README ([#201](https://github.com/pockerman/bitrl/issues/201)) ([e9c461f](https://github.com/pockerman/bitrl/commit/e9c461f3b8d56146ff7e23f6d1c124c4aebdeee9))

## [5.1.1](https://github.com/pockerman/bitrl/compare/v5.1.0...v5.1.1) (2026-11-20)


### Bug Fixes

* **algorithms:** SARSA takes the next action from the next state ([#199](https://github.com/pockerman/bitrl/issues/199)) ([ce2d590](https://github.com/pockerman/bitrl/commit/ce2d5905593ed1fec410f853f11806b8d6912c8b))
* expected sarsa ([#198](https://github.com/pockerman/bitrl/issues/198)) ([4647e91](https://github.com/pockerman/bitrl/commit/4647e9182bff2e6c7750436cb59098fa1026681d))

## [5.1.0](https://github.com/pockerman/bitrl/compare/v5.0.0...v5.1.0) (2026-11-20)


### Features

* mcts line following ([#196](https://github.com/pockerman/bitrl/issues/196)) ([e92a059](https://github.com/pockerman/bitrl/commit/e92a059485c611778c54b8563300134e3b5afaea))

## [5.0.0](https://github.com/pockerman/bitrl/compare/v4.0.0...v5.0.0) (2026-11-20)


### ⚠ BREAKING CHANGES

* building needs bitsim 3.0.0 or later.

### Features

* add envs ([#181](https://github.com/pockerman/bitrl/issues/181)) ([0acbe78](https://github.com/pockerman/bitrl/commit/0acbe7844553daed0ccc6fda9d3dc429c9fbe7ae))


### Bug Fixes

* fix compiler warnings ([#190](https://github.com/pockerman/bitrl/issues/190)) ([6ec6ece](https://github.com/pockerman/bitrl/commit/6ec6eced8a8c239b3138e254ff9d533a02b013f3))
* port the unused headers that were left on the cubeai API ([#195](https://github.com/pockerman/bitrl/issues/195)) ([39e942f](https://github.com/pockerman/bitrl/commit/39e942f10e495f199b1d07377294de9c2d34beac))


### Documentation

* fix the pages of examples 8, 13 and 14 ([#182](https://github.com/pockerman/bitrl/issues/182)) ([82df9e6](https://github.com/pockerman/bitrl/commit/82df9e6db4b2e333973436930329b1f6ddece1e8))
* rewrite the page of example 14 ([#184](https://github.com/pockerman/bitrl/issues/184)) ([12a0fe5](https://github.com/pockerman/bitrl/commit/12a0fe5b0a3b49fa5dcf2ca2b3fbe693e0b8683c))


### Build

* require bitsim 3 ([#183](https://github.com/pockerman/bitrl/issues/183)) ([64bb1cd](https://github.com/pockerman/bitrl/commit/64bb1cd6e1d305c786d5f686b15dc092646bb633))
