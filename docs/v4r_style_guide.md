# 1. Naming
## 1.1. Files

All files should be *under_scored*.

* Header files have the extension `.h`
* Templated implementation files have the extension `.hpp`
* Source files have the extension `.cpp`


## 1.2. Directories

All directories and subdirectories should be under_scored.

* Header files should go under `include/v4r/module_name/.`
* Templated implementation files should go under `include/v4r/module_name/impl/.`
* Source files should go under `src/.`

## 1.3. Includes

All include statement are made with <chevron_brackets>, e.g.:
```cpp
#include <pcl/module_name/file_name.h>
#incluce <pcl/module_name/impl/file_name.hpp>
```
Also keep the list of includes clean and orderly by first including system level, then external libraries, then v4r internal.
I.e. as a general rule: the more general include files go first. Sort includes within a group alphabetically.

## 1.4. Defines & Macros & Include guards

Macros should all be *ALL_CAPITALS_AND_UNDERSCORED*.
To avoid the problem of double inclusion of header files, guard each header file with a `#pragma once` statement placed just past the license.
```cpp
// the license
#pragma once
// the code

## 1.5. Namespaces
Put all your code into the namespace *v4r* and avoid using sub-namespaces whenever possible.

```cpp
namespace v4r {
    ...
}
```

## 1.6. Classes / Structs

Class names (and other type names) should be *CamelCased* . Exception: if the class name contains a short acronym, the acronym itself should be all capitals. Class and struct names are preferably nouns: `PFHEstimation` instead of `EstimatePFH`.

Correct examples:
```cpp
class ExampleClass;
class PFHEstimation;
```

## 1.7. Functions / Methods

Functions and class method names should be *camelCased*, and arguments are *under_scored*. Function and method names are preferably verbs, and the name should make clear what it does: `checkForErrors()` instead of `errorCheck()`, `dumpDataToFile()` instead of `dataFile()`.

Correct usage:
```cpp
int
applyExample (int example_arg);
```
## 1.8. Variables

Variable names should be *under_scored*.
```cpp
int my_variable;
```
Give meaningful names to all your variables.

## 1.8.1. Constants

Constants should be *ALL_CAPITALS*, e.g.:

```cpp
const static int MY_CONSTANT = 1000;
```

## 1.8.2. Member variables

Variables that are members of a class are *under_scored_*, with a trailing underscore added, e.g.:
```cpp
int example_int_;
```

# 2. Indentation and Formatting

V4R uses the [**Google style guide**](https://google.github.io/styleguide/cppguide.html) (with some minor allowed modifations stated in [.clang-format](../.clang-format)). To apply the style guide in your IDE, please make sure you are using clang v5.0 and do the following:
## CLion:
You can use External Tools in CLion.

Go to `File->Settings->Tools->External Tools` and click on the plus sign.

A window should pop up. Use a name of your choice.

For the Tool settings tab, use this configuration:

- Program: clang-format
- Parameters: -i -style=file $FileName$
- Working directory: $FileDir$

Now, with your file open, you can go to `Tools->External tools` and run the config above. It basically calls clang-format and does inplace formatting.

You can also set a custom keymap to it, just search the name of your external tool in the Settings menu.
## QtCreator:
Use the beautifier plugin (should be integrated in QtCreator 3.*.*). To enable it, go to "Help -> About plugins..." and there check "Beautifier" box in "C++" group.
Then set the beautifier tool to use the Google style. Go to "Tools -> Options -> Beautifier", then go to Clang Format  and select "Use predefined style: FILE". OK

## Terminal
To re-format existing files, you can use V4R `clang-format` script. Just go to your V4R root directory and run
```
./scripts/dev/clang-format-all apps/ modules/ samples/
```

Note that this requires clang-format version 5 to be installed.

# 3. Structuring
## 3.1. Classes and API

For most classes in V4R, it is preferred that the interface (all public members) does not contain variables and only two types of methods:

* The first method type is the get/set type that allows to manipulate the parameters and input data used by the class.
* The second type of methods is actually performing the class functionality and produces output, e.g. compute, filter, segment.

## 3.2. Passing arguments

For getter/setter methods the following rules apply:

* If large amounts of data needs to be set it is preferred to pass either by a *const* reference or by a *const* boost shared_pointer instead of the actual data.
* Getters always need to pass exactly the same types as their respective setters and vice versa.
* For getters, if only one argument needs to be passed this will be done via the return keyword. If two or more arguments need to be passed they will all be passed by reference instead.

For the compute, filter, segment, etc. type methods the following rules apply:

*  The output arguments are preferably non-pointer type, regardless of data size.
* The output arguments will always be passed by reference.

## 3.3 Use const
To allow clients of your class to immediately see which variable can be altered and which are used for read only access, define input arguments *const*. The same applies for member functions which do not change member variables.

## 3.3 Do not clutter header files
To reduce compile time amongst others, put your definitions into separate .cpp or .hpp files. Define functions inline in the header only when they are small, say, 10 lines or fewer.

## 3.4 Fix warnings
To keep the compiler output non-verbose and reduce potential conflicts, our CI server will throw errors for warnings. Therefore, please fix all your warnings before committing your files.

## 3.5 Check input range and handle exceptions
To avoid confusing runtime errors and undefined behaviour, check the input to your interfaces for potential conflicts and provide meaningful error messages. If possible, try to catch these exceptions and return in a well-defined state.

## 3.6 Document
V4R uses [Doxygen](https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html) for documentation of classes and interfaces. Any code in V4R must be documented using Doxygen format.

## 3.7 Template your classes/functions whenever appropriate
As many classes and functions in V4R depend on e.g. templated point clouds, allow your classes to accommodate with the various types by using template classes and functions.
