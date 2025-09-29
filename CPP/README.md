# C++ Basics

## C++17
- `if constexpr`: Compile-time branching
- `std::optional`: Safe handling of missing values
- Structured Bindings: Cleaner tuple/pair unpacking
- Parallel Algorithms: Execution policies for speed-up

## C++20
- Concepts: Constrained templates
- Ranges: Functional-style pipelines
- Coroutines: Async & generators
- Modules: Faster builds, no headers


## How a C++ code runs 

![](docs/flow.webp)

- Preprocessor
  
  Preprocessor converts Source code to Expanded code.

    -  removes the comments from the program 
    -  Expands the preprocessor directives such as macros or file inclusion

        - #define PI 3.14 would be replaced with 3.14
        - #include <iostream> when expanded would be replaced by the actual code present in the file iostream

- Compiler

   Compiler converts the Expanded code to Assembly code.

        - checks the program for syntax errors
        - if no error is found , convert the exapded code to assembly code  

- Assembler

  Assembler converts the Assembly code to Object code.


- Linker

  Linker converts the Object code to Executable code.


- Loader

  Finally, the Loader loads the executable file into memory.

