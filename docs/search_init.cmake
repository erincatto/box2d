# Doxygen 1.18 starts the search box only from a doxygen-config block that it writes with the top tab bar.
# DISABLE_INDEX drops the tab bar and the block with it, so the search field never gets its key handlers.
# Start the search directly when the block is missing.
# Run with -DSEARCHJS=<path to html/search/search.js>

file(READ "${SEARCHJS}" text)

set(fallback "document.addEventListener('DOMContentLoaded', () => { if (!document.getElementById('doxygen-config')) init_search(); });")

string(FIND "${text}" "function init_search()" found)
if(found LESS 0)
	message(FATAL_ERROR "search_init.cmake: init_search not found in ${SEARCHJS}, doxygen changed it")
endif()

string(FIND "${text}" "${fallback}" patched)
if(patched LESS 0)
	file(APPEND "${SEARCHJS}" "\n${fallback}\n")
endif()
