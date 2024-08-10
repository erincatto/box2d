find_package(Doxygen REQUIRED dot)

set(DOXYGEN_PROJECT_NAME "Box2D")
set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_USE_MATHJAX YES)
set(DOXYGEN_MATHJAX_VERSION MathJax_3)
set(DOXYGEN_MATHJAX_FORMAT SVG)
set(DOXYGEN_EXTRACT_ALL NO)
set(DOXYGEN_FILE_PATTERNS *.h)
set(DOXYGEN_ENABLE_PREPROCESSING YES)
set(DOXYGEN_MACRO_EXPANSION YES)
set(DOXYGEN_EXPAND_ONLY_PREDEF YES)
set(DOXYGEN_PREDEFINED B2_API= B2_INLINE=)
set(DOXYGEN_WARN_IF_UNDOCUMENTED YES)

# In multiline comments, this takes the first line/sentence as a brief description to use in the table of functions.
# So I don't need to use @brief tags to separate the short description from the full description.
set(DOXYGEN_JAVADOC_AUTOBRIEF YES)

set(DOXYGEN_IMAGE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/images")
set(DOXYGEN_HTML_EXTRA_STYLESHEET "${CMAKE_CURRENT_SOURCE_DIR}/extra.css")
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${CMAKE_CURRENT_SOURCE_DIR}/overview.md")
set(DOXYGEN_PROJECT_LOGO "${CMAKE_CURRENT_SOURCE_DIR}/images/logo.svg")
set(DOXYGEN_LAYOUT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/layout.xml")

set(DOXYGEN_INLINE_SIMPLE_STRUCTS YES)
set(DOXYGEN_TYPEDEF_HIDES_STRUCT YES)
# set(DOXYGEN_DISABLE_INDEX YES)
set(DOXYGEN_GENERATE_TREEVIEW YES)
set(DOXYGEN_FULL_SIDEBAR NO)

# force dark mode to work with extra.css
set(DOXYGEN_HTML_COLORSTYLE DARK)

# this tells doxygen to label structs as structs instead of classes
set(DOXYGEN_OPTIMIZE_OUTPUT_FOR_C YES)
set(DOXYGEN_WARN_IF_INCOMPLETE_DOC NO)

doxygen_add_docs(doc
                "${CMAKE_SOURCE_DIR}/include/box2d"
                "overview.md"
                "hello.md"
                "samples.md"
                "foundation.md"
                "collision.md"
                "simulation.md"
                "loose_ends.md"
                "reading.md"
                "faq.md"
                "migration.md"
                ALL
                COMMENT "Generate HTML documentation")
