# API Files

These JSON API files are updated with each minor version bump. So they can be out of date with main.
You can use `api_dump.py` to generate them otherwise.

- `box2d_api.json` is the default single precision build.
- `box2d_api_double.json` is the `BOX2D_DOUBLE_PRECISION` build. Bind against the file that matches
  the library.

The schema is documented at the top of `api_dump.py`.

## Regenerating

From the repository root in a bash shell. This needs Doxygen and Python 3. The script reads the
Doxygen XML, and `--check-library` compares the result with what a shared library of the same
precision actually exports.

```bash
# Doxygen XML for both precisions
cmake -S . -B build/api_docs -DBOX2D_DOCS=ON -DBOX2D_SAMPLES=OFF -DBOX2D_UNIT_TESTS=OFF
cmake --build build/api_docs --target doc
( cat build/api_docs/docs/Doxyfile.doc
  echo "PREDEFINED += BOX2D_DOUBLE_PRECISION"
  echo "GENERATE_HTML = NO"
  echo "OUTPUT_DIRECTORY = ../build/api_docs/docs_double" ) | ( cd docs && doxygen - )

# Shared libraries to check against
cmake -S . -B build/api_float -DBUILD_SHARED_LIBS=ON -DBOX2D_SAMPLES=OFF -DBOX2D_UNIT_TESTS=OFF
cmake --build build/api_float --config Release --target box2d
cmake -S . -B build/api_double -DBUILD_SHARED_LIBS=ON -DBOX2D_DOUBLE_PRECISION=ON -DBOX2D_SAMPLES=OFF -DBOX2D_UNIT_TESTS=OFF
cmake --build build/api_double --config Release --target box2d

python api/api_dump.py --xml build/api_docs/docs/xml --out api/box2d_api.json --check-library build/api_float
python api/api_dump.py --xml build/api_docs/docs_double/xml --out api/box2d_api_double.json --check-library build/api_double
```

The check fails if a function in the dump is not exported, if an inline function is exported, or if
an exported `b2` symbol is missing from the dump. A missing symbol usually means a public function
has no doc comment, since Doxygen leaves undocumented functions out of the XML.
