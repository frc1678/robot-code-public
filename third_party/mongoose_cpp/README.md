How to update `mongoose-cpp`
============================

1. Clone mongoose-cpp from `https://github.com/Gregwar/mongoose-cpp`.
2. Delete all files except the files in `mongoose/`, `mongoose.c` and
   `mongoose.h`.
3. All the files from the above step should be placed in `upstream`. Look at
   the current upstream folder for an example of this.
4. Clone `https://github.com/open-source-parsers/jsoncpp`.
5. Move the `include/json/` directory into `upstream/`.
6. Replace includes that use angle brackets for local files with quotes. There
   are only a few of these, mostly mongoose and json.
7. There is no step 7.
