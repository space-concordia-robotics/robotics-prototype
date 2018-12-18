# Bash hook to prepend branch name
This explains how to use and setup a git hook that will prepend the issue number
to a commit message.git hooks are used to automate git commands and functions. There are a number of different options, but this particular hook will add the issue number to your "git commit-m" message once it is pushed to the repository. Git hooks live in the git file of each repo, so initializing this hook in the robotics git will not change any other repos you might have.

## Setting up a git hook
In the space club repo directory find the git folder and open hooks contained there in. For robotics the address is:

/<your-loctation-of-repo>/robotics-protoype/git/hooks

once there open prepare-commit-msg.sample in your editor. Paste all the script from the prepender file over the sample code. then save the file and remove the .sample in the file name. that's it, the hook is up and running.

## Using the commit hook prepender

Now, whenever you using the git commit-m command the hook will prepend the issue number to your message. Note that you wont see this in bash or terminal, instead the issue will show up in the repo as [#<issue number>]. This will work as long as your branches are named using our branch naming standards of <word>-<word>-<issue number> .
