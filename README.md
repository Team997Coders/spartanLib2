# spartanLib2

<div align="center">
<a href="https://search.maven.org/artifact/org.chsrobotics.lib/spartanLib2"><img src="https://img.shields.io/maven-central/v/org.chsrobotics.lib/spartanLib2.svg?label=Maven Central"></a>
<a href="https://javadoc.io/doc/org.chsrobotics.lib/spartanLib2"><img src ="https://javadoc.io/badge2/org.chsrobotics.lib/spartanLib2/release_docs.svg?label=Release Documentation"></a>

<a href="https://github.com/Team997Coders/spartanLib2/actions/workflows/main.yml"><img src="https://github.com/Team997Coders/spartanLib2/actions/workflows/main.yml/badge.svg?branch=main"></a>
<a href="https://team997coders.github.io/spartanLib2"><img src="https://github.com/Team997Coders/spartanLib2/actions/workflows/docs.yml/badge.svg"></a>

<a href="https://www.gnu.org/licenses/gpl-3.0"><img src="https://img.shields.io/badge/License-GPLv3-blue.svg"></a>

Reusable robot code for FIRST Robotics Competition Team 997.

<img src="https://github.com/Team997Coders/spartanLib2/raw/main/logo.jpeg" width="350" height="350" />

<a href=https://www.chsrobotics.org>chsrobotics.org</a>

<p><p>
<a href=https://github.com/Team997Coders/spartanLib2/wiki> Wiki</a>

</div>
## Contents:
Currently, the library contains 8 sub-packages:

- `commands`: Command Framework Abstractions
- `controllers`: Feedback Controllers
- `drive`: Differential Drive Modes
- `input`: Joystick Wrappers
- `math`: Filters, Utility Operations
- `telemetry`: Logging
- `trajectory`: Trajectory Generation, Motion Profiles
- `util`: Utility

Future expansion will likely create a `hardware` package, containing abstractions for various hardware APIs for FRC, and a `subsystem` package for WPILib subsystems.


## Contributing:
If you're a member of the Team997Coders org, you can simply create a branch inside this repo and make a pull request to `dev` when you're finished. Currently, 1 review is required and checks must pass to merge to `dev`. No direct commits to `main` are allowed.

For people outside of the org, follow the same steps, except with your own fork. We'll accept any useful PRs!

### Standards:
There aren't stringent criteria for merging, but your code should, at the minimum, be:

- Well-documented (javadoc, variable/method names, source comments explaining complex logic)
- Robustly-tested (not much hardware can be tested without the NI/WPILib HAL, but those should be benchtop tested, and everything else should have tests for many cases written)

## Note to Future Programmers:
In order to keep this library from going the way of the dinosaurs (and spartanLib1...), it can't stay static, not useful to anyone. If there's some bit of code you just *need* to make your life in FRC programming easier, put it in. If there's something in here not being useful, consider removing it.

### Future Leads:
Ask for admin rights from your software mentor so that you can play a large role in future development. If you have issues getting this role, please reach out to the original author(s).

## License:
Licensed under the GNU GPLv3.
