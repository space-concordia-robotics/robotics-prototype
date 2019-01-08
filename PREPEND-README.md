# Bash hook to prepend branch name
This explains how to use and setup a git hook that will prepend the issue number
to a commit message. Git hooks are used to automate git commands and functions. There are a number of different options, but this particular hook will add the issue number to your "git commit-m" message once it is pushed to the repository. Git hooks live in a local git/hooks file of each repo, so initializing this hook in the robotics-prototype repo will not change any other repos you might have.

## Setting up a git hook
Windows:
In your space concordia directory find the git folder and open hooks contained there in. It should look like this:

robotics-protoype/git/hooks

once there open prepare-commit-msg.sample in your editor. Replace the code in the sample with the code found in commit-message-hook.sh. Save the file by keeping the name the same and removing the .sample suffix.

Linux:

Open the .git/hooks folder. Should be in this directory:

```
robotics-protoype/.git/hooks
```
once there copy we need to make a copy of the prepare-commit-msg.sample :
```
$touch prepare-commit-msg.sample
```
copy contents of the sample to our new file:
```
$cp cp prepare-commit-msg.sample prepare-commit-msg
```
make the new file executable:
```
chmod +x prepare-commit-msg
```

Now open this file with your editor of choice and replace the code with that in commit-message-hook.sh. Save the file. The hook should now be set up.


### Using the commit hook prepender

Now, when ever you using the git commit-m command the hook will prepend the issue number to your message. This will show up in the repo as [#<issue number>], so there is no longer a need to add this to your commit message. This will work as long as your branches are named using our branch naming standards defind in our wiki, otherwise the commit will be aborted.

in order to write a long commit message using git commit -m, write a concise title and then press enter twice before closing the quotation marks. Then, type as long a message as is appropriate and close the quation mark.

Finish the commit and push as usual.
