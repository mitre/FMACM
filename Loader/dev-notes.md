# Loader Library Dev Notes

This library is comprised entirely of legacy code. The library reads text files in that are a specific format. The code was auto-generated using a tool. All knowledge of that process and who did it are lost. As developers, our goal is to understand the behavior of this library and maintain it as little as possible. It suffices for our needs for now, but should not be expanded. The only reason to open this code is to fix bugs. Otherwise we leave it alone.

## Format

The semicolon `;` is a comment character and can occur anywhere in a line. Any text on a line that occurs to the right of the semicolon is ignored during loading.

All white space is ignored. White space can be tab or space characters.

Keyword-Variable inputs occur on a single line like this:
```
keyword value
```

Complex inputs are separated into their own area using curly braces like this:
```
complex_stuff {
   keyword1 value1
   keyword2 value2
}
```
Long files can be split out into multiple files using `#include path/to/file.txt` syntax like this:
```
keyword value
complex_stuff {
   keyword1 value1
   keyword2 value2
   #include more_values.txt
}
```