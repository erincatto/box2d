/*
* Copyright (c) 2013 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
#ifndef BOX2D_BODY_TRACKER_H
#include "Box2D/Box2D.h"
#include <string>
#include <map>
#include <utility>
#include <vector>

class b2Body;

/// BodyTracker: a class to help in the tracking of b2Body data
/// This module exists specifically for helping in the tracking of
/// b2Body simulation data during unit tests.  The tracker will
/// simply keep track of the specified values in a set of b2Body objects
/// over a single simulation and compare that runs values to that
/// of an earlier one, as found in a baseline file.
///
/// For a usage example, please look in HelloWorldTests.cpp in this
/// directory.
///
class BodyTracker {
public:
	// Track flags to state what properties to track.
	// Note to developers: If you make an addition to this, you must also
	// make an addition to internal function that matches this flag to a
	// string.
	enum TrackFlags {
		TRACK_POSITION = 1 << 0,
		TRACK_ANGLE =    1 << 1,
		TRACK_WORLD_CENTER = 1 << 2,
		TRACK_LOCAL_CENTER = 1 << 3,
		TRACK_LINEAR_VELOCITY = 1 << 4,
		TRACK_ANGULAR_VELOCITY = 1 << 5
	};

	// Constructor
	// @param baselineFile The name of the file in which are the results to be
	// compared.
	// @param outputFile The output file in which the data is always written.
	// @param flags The sum of TrackFlags values specifying the properties to
	// track.
	BodyTracker(const std::string &baselineFile, const std::string &outputFile,
				int32 flags);
	virtual ~BodyTracker();

	// Called to register that we will be tracking a body
	void TrackBody(const b2Body *body, const std::string &name);

	// Called from the inner "Step" loop
	bool TrackStep(const b2Body *body, float32 timeStep);

	// Called when tracking is started.
	bool BeginTracking();

	// Called when tracking is done.
	bool EndTracking();

	// This call will check ALL the tracked values against the baseline's
	bool CompareToBaseline(const b2Body *body, int32 flag,
							 float32 epsilon) const;

	// Get strings representing the errors if any of the Compare() functions
	// return false.
	const std::vector<std::string> &GetErrors() const;

public:
	// Set the root directory for baseline and output files using argv[0]
	// (the program's path).
	static void SetWorkingDirectory(const char *argv0);

private:
	bool ReadBaseline();
	std::string TrackFlagsToString(int32 flag) const;

	// Disable the assignment operator.
	BodyTracker& operator = (BodyTracker& rhs)
	{
		B2_NOT_USED(rhs); return *this;
		}

	bool m_tracking;
	bool m_baselineRead;
	const int32 m_flags;
	std::map<const b2Body *, std::string> m_bodyNames;
	const std::string m_baselineFile;
	const std::string m_outputFile;

	struct Sample {
		float32 timeStep;
		b2Vec2 position;
		float32 angle;
		b2Vec2 worldCenter;
		b2Vec2 localCenter;
		b2Vec2 linearVelocity;
		float32 angularVelocity;
	};
	typedef std::vector<Sample> SampleVec;

	std::map<const b2Body *, SampleVec> m_samples;

	std::map<const std::string, SampleVec> m_baselineSamples;
	int32 m_baselineFlags;

	mutable std::vector<std::string> m_errors;

	// Root directory for baseline and output files.
	static std::string s_baselineRootDir;
};
#endif
