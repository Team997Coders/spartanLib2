# spartanLib2
[![CI](https://github.com/Team997Coders/spartanLib2/actions/workflows/main.yml/badge.svg?branch=dev)](https://github.com/Team997Coders/spartanLib2/actions/workflows/main.yml)
[![Deploy Javadoc](https://github.com/Team997Coders/spartanLib2/actions/workflows/docs.yml/badge.svg)](https://github.com/Team997Coders/spartanLib2/actions/workflows/docs.yml)

Reusable code for FRC team 997.

<img src="https://lh4.googleusercontent.com/zyjl2m5D_WEY1cgqMgGPeLXmayvhxZGDu_uWI15sOP038dfpa6T9SY9PhvHk7LDcsAIKJceWxr8r61DqQBFuQ2_QzUlhqz2WqYc192rUr94EsJ6xLmmsELLkoPasVJAxwQ=w1280" alt="997-logo" width="350">

https://www.chsrobotics.org/

## Note to Future Programmers:
In order to keep this library from going the way of the dinosaurs (and spartanLib1...), it can't stay static, not useful to anyone. If there's some bit of code you just *need* to make your life in FRC programming easier, put it in. If there's something in here not being useful, consider removing it.

### Future Leads:
Ask for admin rights from your software mentor so that you can play a large role in future development. If you have issues getting this role, please reach out to the original author(s).

## Contents:
Currently, the library contains 4 sub-packages:

- `math`: Filters, Interpolation, Utility operations
- `telemetry`: Logging
- `trajectory`: Motion Profiles
- `util`: Utility

Future expansion will likely create a `hardware` package, containing abstractions for various hardware APIs for FRC, and `command` and `subsystem` packages for the WPILib CommandBased paradigm.


## Contributing
If you're a member of the Team997Coders org, you can simply create a branch inside this repo and make a pull request to `dev` when you're finished. Currently, 1 review is required and checks must pass to merge to `dev`. No direct commits to `main` are allowed.

For people outside of the org, follow the same steps, except with your own fork. We'll accept any useful PRs!

### Standards:
There aren't stringent criteria for merging, but your code should, at the minimum, be:

- Well-documented (javadoc, variable/method names, source comments explaining complex logic)
- Robustly-tested (not much hardware can be tested without the NI/WPILib HAL, but those should be benchtop tested, and everything else should have tests for many cases written)
