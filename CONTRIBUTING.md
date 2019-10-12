# Programming Guidelines

## General Guidelines

### Why do we even need guidelines in the first place?

At the beginning, Space Concordia Robotics was small and the code base was maintained by two individuals that knew their code inside out. The robotics team is now growing with members of a wide range of programming experience. Due to the negligence of the best practices, the code became spaghetti and almost impossible to read for new members. It is hard to debug and to test. We need clean code because

#### Clean code is easier to read

#### Clean code is easier to debug and is more maintainable

#### Well designed code is scalable

#### Well planned code is fun to build on

#### There is a cost to having a total mess
Teams that have moved swiftly at the beginning of projects can find themselves moving at a much slower rate further in. The term [Technical Debt](https://en.wikipedia.org/wiki/Technical_debt) describes the cost of the additional work that needs to be done (because of slower debugging, more bugs, etc) due to messy code.

## Best Practices

### Functions

#### Do one thing

Functions should do one thing. They should do it well. They should do it only. One symptom of doing more than one thing is having sections within functions.

#### Size

Size does matter! The size of the function should be small. A function that cannot be read in one screen should definitely be split up.

#### Duplication

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


### Coupling
<<<<<<< HEAD

#### Modularity

#### Levels of abstraction

### Comments

#### Rotted Comments
=======
Coupling is the degree of interdependence between software modules.

#### Modularity
Since the rover needs to be modular, we need to have very loose coupling. The software needs to have independent modules that we can quickly swap out. This is one of the many reasons we use ROS, which creates a modular foundation.

#### Levels of abstraction
it is a best practice to abstract the code into different layers. For example, code related to communication with ROS should not include GUI changes. Common layers are GUI, Logic, Communication.


### Comments
Comments can be a blessing or a cure. Proper documentation can help developers while bad documentation can obstruct or mislead them.

#### Useless comments
Ideal code is self explanatory. Do not comment when it is obvious like

```
//Initialize ROS
initRos();
```

This adds lines for the programmers to read without adding any value.

#### Rotted Comments
One major issue with comments is that they rot. Before writing a comment, make sure that this comment will still be relevant after the code passes through a lot of changes.
>>>>>>> f11aa33... Add section on comments

### Meaningful names

With the function that we improved above there is still a glaring issue :

```
void Commands::setTto(bool throttleTimeOut) {
    String msg = "ASTRO Throttle Timeout " + String(throttleTimeOut ? "On" : "Off");
    println(msg);

}
```

What the heck is `TTO`? Observing this function and argument, it's obvious that it's for `Throttle Timeout`. However, imagine that you were stumbling upon a call to this function like this

```
//some code
setTto(false);
//more code
```
To know what `TTO` means, you need to go to the function definition which could be tedious if you have to do this often.

Furthermore, you'd expect `throttleTimeOut` to be a number, such as the timeout time, but it is a boolean.

```
void Commands::setThrottleTimeout(bool hasThrottleTimeout) {
    String msg = "ASTRO Throttle Timeout " + String(hasThrottleTimeout ? "On" : "Off");
    println(msg);
}
```

Boolean variable should start with `has` or `is`.


```
//some code
setThrottleTimeout(false);
//more code
```
### Error Handling
