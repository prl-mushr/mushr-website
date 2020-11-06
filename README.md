# mushr-website

The MuSHR website is generated using the Hugo static site generator. Please
refer to Hugo's documentation for any major changes. Install Hugo v0.61.0 to
match the deployment pipeline version!

After installing Hugo, you can test the site locally by cloning this repository
and running `hugo server`. If you have draft content that you want to generate,
add the `-D` or `--buildDrafts` flag.

**Talk to @schmittle before pushing major changes.**

## Deploying website edits

We use [GitHub Actions](.github/workflows/pages.yaml) to automatically deploy
the website to GitHub Pages (the `gh-pages` branch of `prl-mushr.github.io`).
Create a new Git branch with your edits and make a pull request when you're
ready to publish.

## Editing content

All website content can be found in the `content` folder. Update one of the
existing Markdown files, or add your own.

To start a new tutorial, run `hugo new tutorials/<tutorial-name>.md`. This will
create the `content/tutorials/<tutorial-name>.md` file and fill in some parts
for you.
