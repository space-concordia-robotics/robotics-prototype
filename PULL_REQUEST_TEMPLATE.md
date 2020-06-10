# Assignee Section

## Description
[Write a summary of the changes included in this Pull Request]

[If applicable, include a list of main points]
- Example bug fixed.
- Added example function with the responsibility of example.

### Steps for Testing
[Imagine you are the reviewer and did not write the code. Write all the steps necessary to test the feature(s) in an **ordered list**. Be **specific** and include important details here so testers don't need to jump through hoops to test your pull request. Be explicit in all the steps, and make sure the expected output is clearly stated.

Bad:
- [...]
- Open the dev console and call the different logging functions [...].
- Compare with how appendToConsole() behaves."

The above example is bad because it is too unspecific. What are the different logging functions called, how many are there to test specifically? How should the behavior be compared, should the results behave similarly or differently?

Good:
1. [...]
2. [...]
3. Call one of the different logging functions (logDebug, logInfo, logWarn, logErr, logFatal).
4. Compare with how the appendToConsole() method behaves, they should print to the same consoles according to the boolean flags passed, though appendToConsole should not generate any roslogs.
5. Use rostopic echo /rosout to monitor roslogs.
6. Repeat this for all logging functions.

The improved example makes any doubts the reviewer may have clear, also the list is numbered so the reviewer can easily refer to which step fails if they catch one.]

closes #[Insert your issue number here and **remove the square brackets**]

The approval from all software team leads is necessary before merging.

# Reviewer Section

Aside from local testing and the General Integration Test it is implied that static analysis should be included in the verification process.

- [ ] Local Test Performed Successfully
- [ ] [General Integration Test](https://docs.google.com/document/d/1ug0CpA1cIzURP8DDFSvCt2CEJJSwJ6Ta6B1LG_hYk6I/edit) Performed Successfully

For Pull Requests that do not include code changes, it is not required to perform the tests above.
