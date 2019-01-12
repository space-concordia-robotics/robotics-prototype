# Bash hook to prepend branch name
This explains how to use and setup a git hook that will prepend the issue number
to a commit message. Git hooks are used to automate git commands and functions. There are a number of different options, but this particular hook will add the issue number to your ```git commit-m``` message once it is pushed to the repository. Git hooks live in a local git/hooks file of each repo, so initializing this hook in the robotics-prototype repo will not change any other repos you might have.

## Setting up a git hook
Windows:
In your space concordia directory find the git folder and open hooks contained there in. It should look like this:
```
robotics-protoype/git/hooks
```

Once there open ```prepare-commit-msg.sample``` in your editor. Replace the code in the sample with the code found in ```commit-message-hook.sh.``` Save the file by keeping the name the same and removing the .sample suffix.

Linux:

From the root of the repository:
```
cp commit-message-hook.sh .git/hooks/prepare-commit-msg
```

### Using the commit hook prepender

Now, when ever you using ```git commit-m``` the hook will prepend the issue number to your message. This will show up in the repo as [#<issue number>], so there is no longer a need to add this to your commit message. This will work as long as your branches are named using our branch naming standards defind in our wiki, otherwise the commit will be aborted.

In order to write a long commit message using ```git commit -m```, write a concise title and then press enter twice. Then, type as long a message as is appropriate and close the quation mark. This ensures it will be formatted nicely on github.

Finish the commit and ```git push``` as usual.
