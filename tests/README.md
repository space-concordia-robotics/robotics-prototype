# Tests
This is where we keep our tests and group them according to type.
So far we have some sample unit tests, and an empty folder for feature tests.

## Unit tests
Testing an individual unit, such as a method (function) in a class, with all dependencies mocked up.

## Functional tests
AKA Integration Test, testing a slice of functionality in a system. This will test many methods and may interact with dependencies like Databases or Web Services.

## Benchmark tests
Tests in order to assess the relative performance of an object, normally by running a number of standard tests and trials against it.
These tests are usually associated with assessing performance characteristics of computer hardware.

# Prototype tests
Tests developed during prototyping.

# ModuleNotFoundError
This issue with module imports via `pytest` was the motivating factor to change the project directory structure. For this technique to work, the 'source' code must live inside (nested) a main directory (usually named the same as project directory name or other suitable representative identifier such as **robot** in this case). The `src` subdirectory was renamed because it made no sense when importing a package module by name like `import src.basestation.Motor`, which has no meaning/place in a module semantic context (`import robot.basestation.Motor` is much more appropriate). Most Python projects do not use a `src` directory unless it's for storing their source code that eventually gets compiled to binary (i.e. such as `.c`, `.h`, etc.. files). Also, `base-station` was renamed to `basestation` because Python no-likey dashes in import statements.


