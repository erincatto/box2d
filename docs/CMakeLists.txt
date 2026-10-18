# https://www.doxygen.nl/manual/markdown.html#markdown_extra
# https://docs.github.com/en/get-started/writing-on-github/getting-started-with-writing-and-formatting-on-github/basic-writing-and-formatting-syntax

find_package(Doxygen REQUIRED)

set(DOXYGEN_PROJECT_NAME "Box2D")
set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_GENERATE_XML YES)
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
# Drop the top tab bar, the tree carries the same navigation
set(DOXYGEN_DISABLE_INDEX YES)
set(DOXYGEN_GENERATE_TREEVIEW YES)
set(DOXYGEN_FULL_SIDEBAR NO)

# Only the ## sections in the tree, deeper headings made the manual pages expand into long lists
set(DOXYGEN_TOC_INCLUDE_HEADINGS 2)

# No per-page outline panel on the right
set(DOXYGEN_PAGE_OUTLINE_PANEL NO)

# Dark theme baked by doxygen, extra.css restyles it in the Box2D grays and accent
set(DOXYGEN_HTML_COLORSTYLE DARK)

# this tells doxygen to label structs as structs instead of classes
set(DOXYGEN_OPTIMIZE_OUTPUT_FOR_C YES)
set(DOXYGEN_WARN_IF_INCOMPLETE_DOC NO)

# Keep enum values out of the summary tables, b2HexColor alone has over a hundred
set(DOXYGEN_ENUM_VALUES_PER_LINE 0)

# No Graphviz diagrams, the struct collaboration graphs only repeat the field types
set(DOXYGEN_HAVE_DOT NO)

doxygen_add_docs(doc
                "${CMAKE_SOURCE_DIR}/include/box2d"
                "overview.md"
                "hello.md"
                "samples.md"
                "foundation.md"
                "collision.md"
                "simulation.md"
                "loose_ends.md"
                "recording.md"
                "large_worlds.md"
                "character.md"
                "reading.md"
                "questions.md"
                "release_notes_v320.md"
                ALL
                COMMENT "Generate HTML documentation")
                

# Jump to anchors instead of doxygen's animated scroll
add_custom_command(TARGET doc POST_BUILD
                COMMAND ${CMAKE_COMMAND} -DNAVTREE=${CMAKE_CURRENT_BINARY_DIR}/html/navtree.js
                        -P ${CMAKE_CURRENT_SOURCE_DIR}/instant_scroll.cmake
                VERBATIM)
