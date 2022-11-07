# spartanLib2
[![Build](https://github.com/Team997Coders/spartanLib2/actions/workflows/main.yml/badge.svg?branch=dev)](https://github.com/Team997Coders/spartanLib2/actions/workflows/main.yml)
[![Javadocs](https://github.com/Team997Coders/spartanLib2/actions/workflows/docs.yml/badge.svg)](https://team997coders.github.io/spartanLib2)

Reusable code for FRC team 997.

<img src="https://scontent.fhio2-2.fna.fbcdn.net/v/t1.18169-9/17952471_1343549849025086_7905389874919796012_n.jpg?_nc_cat=107&ccb=1-7&_nc_sid=09cbfe&_nc_ohc=cukHnJHO-icAX89KacE&_nc_ht=scontent.fhio2-2.fna&oh=00_AfBSIvr6fqBbK5NVnG7MnDfyw8yNqmzuazWEnHRFbrOrJw&oe=638FFDEE" width="350" height="350" />

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
