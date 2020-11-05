# mushr-website

**Use Hugo v0.61.0 to build using the same version of hugo as the pipeline.**

The MuSHR website is generated using the Hugo static site generator. Please refer to Hugo's documentation for any major changes. You can host the site locally by running `hugo server -D` in this directory. <b>Do reach out to @johannesburg or @schmittle before pushing major changes.</b>

## Pushing website edits
There is an [Azure Pipeline](https://dev.azure.com/prl-mushr/mushr-website/_build) that automatically deploys the website on merge to master. Create a branch to make edits in, then merge to master when you want to push.

## Editing content

All website content can be found in the `content` folder. Update one of the
existing Markdown files, or add your own.

To start a new tutorial, run `hugo new tutorials/<tutorial-name>.md`. This will
create the `content/tutorials/<tutorial-name>.md` file and fill in some parts
for you.
