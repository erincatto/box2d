# Doxygen's navtree.js slides to an anchor for up to 500 ms and has no option to turn it off.
# Rewrite the call to jump, keeping the callback that runs the highlight glow.
# Run with -DNAVTREE=<path to html/navtree.js>

file(READ "${NAVTREE}" text)

set(animated "animateScrolling(docContent, pos+dcScrTop-dcOffset, Math.max(50,Math.min(500,dist)), function() {")
set(instant "docContent.scrollTop = pos+dcScrTop-dcOffset; requestAnimationFrame(function() {")

string(FIND "${text}" "${animated}" found)
if(found GREATER_EQUAL 0)
	string(REPLACE "${animated}" "${instant}" text "${text}")
	file(WRITE "${NAVTREE}" "${text}")
else()
	string(FIND "${text}" "${instant}" patched)
	if(patched LESS 0)
		message(FATAL_ERROR "instant_scroll.cmake: anchor scroll code not found in ${NAVTREE}, doxygen changed it")
	endif()
endif()
