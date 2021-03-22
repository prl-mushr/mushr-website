---
title: "{{ replace .TranslationBaseName "-" " " | title }}"
date: {{ .Date }}
summary: "Brief summary of your tutorial."
difficulty: "Beginner, Intermediate, or Advanced"
duration: 0
featured: false  # whether this is listed at / (must also be top 6 by weight). 
active: true     # whether this is listed at /tutorials/
draft: true      # whether Hugo considers this a draft
weight: 3        # 2 = intro tutorial 3 = anything else
---

<h2> By: <a href=https://mushr.io/>Your Name</a></h2>

<!-- Header figure required! -->
<br>
{{< figure src="/tutorials/first_steps/firststep.jpg" width="800" >}} <br>                           
<br>

### Introduction
1 to 2 sentence intro of what your tutorial is all about

### Goal
What is the end product of this tutorial?

### Requirements
List of previous tutorials, hardware requirements, or system requirements to complete this tutorial. Requirements from previous tutorials are implied and do not need to be restated.
* **requirement 1:** details
* **requirement 2:** other details

## Section 1
Make as many sections as you want! But do not name them section (not descriptive) see [quickstart](/tutorials/quickstart) for example.

If explaining bash steps please explain the step in english and then the command.  

Open rviz
{{< highlight bash >}}
$ rviz
{{< / highlight >}}

then launch ros
{{< highlight bash >}}
$ roscore 
{{< / highlight >}}

Refrain from putting multiple lines of commands in one bash box. This should only be done if it is obvious what is happening line to line.

If you are embedding python code blocks please use the following brackets

{{< highlight python "linenos=table" >}}
def print_the_truth():
    print("MuSHR is amazing!")

print_the_truth()
{{< / highlight >}}

## Section 2
When you are done your tutorial, make a PR!

## Going Futher 
For things not necessary in the original tutorial. This helps to keep the base tutorial short. This is a good section for explaining how to tweak settings etc.

## Troubleshooting
When people make edits to your tutorial later, it is good to have this section for them to add some tips to challenges they may face.

* **the joystick isn't working:** Turn on the joystick then try again...
* **the computer won't start:** Check to make sure it is getting power, if not reflash. 
