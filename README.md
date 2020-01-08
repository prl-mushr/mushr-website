# mushr-website

**Use Hugo v0.61.0 to build using the same version of hugo as the pipeline.**

The MuSHR website is generated using the Hugo static site generator. Please refer to Hugo's documentation for any major changes. You can host the site locally by running `hugo server -D` in this directory. <b>Do reach out to @johannesburg or @schmittle before pushing major changes.</b>

## Pushing website edits
There is an [Azure Pipeline](https://dev.azure.com/prl-mushr/mushr-website/_build) that automatically deploys the website on merge to master. Create a branch to make edits in, then merge to master when you want to push.

## Editing content
All website content can be found in the `content` folder. Merely update one of the existing markdown files, or add your own (I recommend merely duplicating an existing one, unless you're making extensive changes).
