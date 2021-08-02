# Template for mrs nodelet

This project consists of:
- script for setting up a new nodelet as a fresh project with some features predefined
- template directory

### What does the script do?
The script:
- clone a project to a new repo
- changes project name, class name and nodelet name in all needed files (default src, default header, package.xml, plugins.xml, launchfile, cmake file)
- setting some extra packages in cmake
- remove previous git folder
- optionally: setting description, author name and email

### There are short and long ways of usage
short: <br>
`./mrs_create_nodelet_template.sh -s project_name namespace_name class_name path` <br>
long: <br>
`./mrs_create_nodelet_template.sh [options]` <br>
available options: <br>
```[bash]
    -h      --help                  Show help message.
    -pn     --project-name          Project name. example_project by default.
    -pd     --project-description   Project description. Write description as a one space-separated string (in columns).
    -cp     --cmake-packages        Used for cmake "find_package". 'roscpp' and 'mrs_lib' are default packages. Write packages as a one space-separated string (in columns). Example: 'std_msgs roscpp mrs_lib'
    -nn     --namespace-name        Namespace
    -cn     --class-name            Nodelet class name
    -an     --author-name           Author name
    -ae     --author-email          Should be cpecified for correct compilation!
    -pp     --project-path          Where to create new project. ./ by default
```
