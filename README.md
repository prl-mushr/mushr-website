# mushr-website

The MuSHR website is generated using the Hugo static site generator. Please
refer to Hugo's documentation for any major changes. 

## Requirements

Install [Hugo](https://gohugo.io/getting-started/installing/) v0.86.0 to match
the deployment pipeline version!

## Running the site locally

You can test the site locally by cloning this repository and running:

`hugo server`

If you have draft content that you want to generate,
add the `-D` or `--buildDrafts` flag.

## How to create a new tutorial 

To create a new tutorial, run:  

`hugo new tutorials/<tutorial-name>.md`  

This will create the `content/tutorials/<tutorial-name>.md` template file for
you to get started. Once you are done, make a PR and set @schmittlema as the
reviewer.

## Editing content

All website content can be found in the `content` folder. Update one of the
existing Markdown files, or add your own. Once done, make a PR and set
@schmittlema as the reviewer.

## Deploying website edits

We use [GitHub Actions](.github/workflows/pages.yaml) to automatically deploy
the website to GitHub Pages (the `gh-pages` branch of `prl-mushr.github.io`).
Create a new Git branch with your edits and make a pull request when you're
ready to publish.

