## Working with Git

Never commit and push to master branch! Always create your own branch before changing things.

Every now and then update remote changes. This command fetches changes from all the remotes. Fetch does not merge code in any way.

`git remote update`

Create feature branch based on origin/master

`git checkout -b feature-1 origin/master`

Do some work and commit

`git commit`

And some more ...

Update code from remotes

`git remote update`

And rebase. Rebase will take code from master and merge latest commits from master into your branch history

`git rebase origin/master`

Push your branch to github

`git push -u origin feature-1`

Do some more work and commit again

`git commit`

Rebase once again

`git remote update`
`git rebase origin/master`

Now we may have to force push

`git push -f`

When your work on branch is complete push to github and create pull request.
