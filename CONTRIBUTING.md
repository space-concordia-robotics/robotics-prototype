# Programming Guidelines

## General Guidelines

### Why do we even need guidelines in the first place?

At the beginning, Space Concordia Robotics was small. The code base was maintained by two individuals that knew the code very well. The robotics team is now growing with new members of a wide range of programming experience. Due to the negligence of the best practices, the code became spaghetti and almost impossible to read for new members. It is hard to debug and to test. We need clean code because

#### Clean code is easier to read

#### Clean code is easier to debug and is more maintainable

#### Well designed code is scalable

#### Well planned code is fun to build on

#### There is a cost to having a total mess
Teams that have moved swiftly at the beginning of projects can find themselves moving at a much slower rate further in. The term [Technical Debt](https://en.wikipedia.org/wiki/Technical_debt) describes the cost of the additional work that needs to be done (because of slower debugging, more bugs, etc) due to messy code.

## Best Practices

These best practices are mostly taken from the book `Clean Code, A Handbook of Agile Craftsmanship` by Robert C. Martin 

### 1. Functions

#### 1.1 Do one thing

Functions should do one thing. They should do it well. They should do it only. One symptom of doing more than one thing is having sections within functions. These can oftentimes easily be split up into subroutines that will only have a specific role.

#### 1.2 Size

Size does matter! The size of the function should be **small**. A function that cannot be read in one screen should definitely be split up.

#### 1.3 Duplication

Duplicating code is one of the primary source of hard to maintain code. Here is an example of duplicated code in the code base of robotics-prototype

```
void Commands::ttoOn(void) {
    throttleTimeOut = true;
    String msg = "ASTRO Throttle Timeout " + String(throttleTimeOut ? "On" : "Off");
    println(msg);

}
void Commands::ttoOff(void) {
    throttleTimeOut = false;
    String msg = "ASTRO Throttle Timeout " + String(throttleTimeOut ? "On" : "Off");
    println(msg);

}
```

Which can just be simplified to

```
void Commands::setTto(bool throttleTimeOut) {
    String msg = "ASTRO Throttle Timeout " + String(throttleTimeOut ? "On" : "Off");
    println(msg);

}
```

What we gain by eliminating duplication is the need to change the function multiple times to make changes. If the logic above need to add another instruction, we would need to change `ttoOn(void)` and `ttoOff(void)` while now we only need to change `setTto(bool)`

There are more changes we could do to improve this function that we will see later.


### 2. Coupling
Coupling is the degree of interdependence between software modules.

#### 2.1 Modularity
Since the rover needs to be modular, we need to have very loose coupling. The software needs to have independent modules that we can quickly swap out. This is one of the many reasons we use ROS, which creates a modular foundation.

#### 2.2 Levels of abstraction
It is a best practice to abstract the code into different layers. For example, code related to communication with ROS should not include GUI changes. Common layers are GUI, Logic, Communication.


### 3. Comments
Comments can be a blessing or a cure. Proper documentation can help developers while bad documentation can obstruct or mislead them.

#### 3.1 Useless comments
Ideal code is self explanatory. Do not comment when it is obvious like

```
//Initialize ROS
initRos();
```

This adds lines for the programmers to read without adding any value.

#### 3.2 Rotted Comments
One major issue with comments is that they rot with time. Before writing a comment, make sure that this comment will still be relevant after the code passes through a lot of changes.

#### 3.3 Todos
Do not write any `Todo` comments. These get easily lost and forgotten. Open an issue instead.
 

### 4. Meaningful Names

#### 4.1 Variable Names

With the function that we improved above there is still a glaring issue :

```
void Commands::setTto(bool throttleTimeOut) {
    String msg = "ASTRO Throttle Timeout " + String(throttleTimeOut ? "On" : "Off");
    println(msg);

}
```

What the heck is `TTO`? By observing this function and it's argument, it is obvious that it's for `Throttle Timeout`. However, imagine that you were stumbling upon a call to this function like this

```
//some code
setTto(false);
//more code
```
To know what `TTO` means, you need to go to the function definition which could be tedious if you have to do this often.

Furthermore, you would expect `throttleTimeOut` to be a number, the timeout time, but it is a boolean.

```
void Commands::hasThrottleTimeout(bool hasThrottleTimeout) {
    String msg = "ASTRO Throttle Timeout " + String(hasThrottleTimeout ? "On" : "Off");
    println(msg);
}
```

Boolean variable should start with `has` or `is`.


```
//some code
hasThrottleTimeout(false);
//more code
```
Now it is much cleaner when the call is being read.

#### 4.2 Magic Values

Magic values (or magic numbers) are values used with unexplained context.

An example of the usage of a magic value :
```
if (isPressed(input[65]))
```

We do not know what `65` is.

To fix this problem, we can simply defined a constant.

```
if (isPressed(input[A_KEY]))
```


Magic values should be avoided and replaced by variables or constants with names that self-describes their behavior.



## Programming Specific Guidelines

### Python

We use the PEP8 Style Guide. Please refer to it [here](https://www.python.org/dev/peps/pep-0008/)

### JavaScript

We are attempting to use a strict variant of JavaScript. We have not enable `strict mode` yet, but all future code should respect the strict variant of JavaScript which can be found [here](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Strict_mode)

We are also using JQuery, therefore you should use JQuery for document traversal and and manipulation. 

JavaScript should be organized per Jinja template.

### C++

While we have not officially started using standard programming practices. Future C++ code should use the C++ core guidelines defined [here](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-introduction)


### HTML

HTML code must be placed in templates using [Jinja](https://palletsprojects.com/p/jinja/). Note that Jinja is included with Flask. Each module should have it's own HTML to meet our modularity requirements.

### CSS
CSS code should be organized per Jinja template along with JavaScript and HTML.

Display values should avoid being hardcoded (such as width for example) as they hurt the ease of responsiveness. Instead, bootstrap columns should be used to adjust width.

CSS should be regrouped together. Inline CSS should not be present.
