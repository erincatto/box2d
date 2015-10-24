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
#include "BodyTracker.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <math.h>
#include <stdlib.h>

#define PRINT_PRECISION 8

std::string BodyTracker::s_baselineRootDir;

BodyTracker::BodyTracker(const std::string &baselineFile,
						 const std::string &outputFile, int32 flags) :
	m_tracking(true),
	m_baselineRead(false),
	m_flags(flags),
	m_baselineFile(s_baselineRootDir + baselineFile),
	m_outputFile(s_baselineRootDir + outputFile)
{
}

BodyTracker::~BodyTracker()
{
}

void BodyTracker::TrackBody(const b2Body *body, const std::string &name)
{
	m_bodyNames[body] = name;
	m_samples[body] = SampleVec();
}

bool BodyTracker::TrackStep(const b2Body *body, float32 timeStep)
{
	Sample sample;
	sample.timeStep = timeStep;

	if (m_flags & TRACK_POSITION) {
		sample.position = body->GetPosition();
	}
	if (m_flags & TRACK_ANGLE) {
		sample.angle = body->GetAngle();
	}
	if (m_flags & TRACK_WORLD_CENTER) {
		sample.worldCenter = body->GetWorldCenter();
	}
	if (m_flags & TRACK_LOCAL_CENTER) {
		sample.localCenter = body->GetLocalCenter();
	}
	if (m_flags & TRACK_LINEAR_VELOCITY) {
		sample.linearVelocity = body->GetLinearVelocity();
	}
	if (m_flags & TRACK_ANGULAR_VELOCITY) {
		sample.angularVelocity = body->GetAngularVelocity();
	}
	m_samples[body].push_back(sample);
	return true;
}

std::ostream& operator<<(std::ostream& out, const b2Vec2& vec)
{
	out << std::setprecision(PRINT_PRECISION) << vec.x << " "
	    << std::setprecision(PRINT_PRECISION) << vec.y;
	return out;
}

bool BodyTracker::BeginTracking()
{
	if (m_flags == 0) {
		return false;
	}
	m_errors.clear();
	m_samples.clear();
	m_baselineSamples.clear();
	m_tracking = true;
	return true;
}

bool BodyTracker::EndTracking()
{
	m_tracking = false;

	// Write out data
	std::ofstream out(const_cast<const char *>(m_outputFile.c_str()));
	if (out.bad()) {
		m_errors.push_back("Error: Could not open file " + m_outputFile);
		return false;
	}

	// Output header.
	out << "# ";
	out << TrackFlagsToString(m_flags);
	out << "\n";

	std::map<const b2Body *, std::string>::iterator it = m_bodyNames.begin();
	for ( ; it != m_bodyNames.end() ; ++it ) {
		const b2Body *body = it->first;
		const std::string &name = it->second;
		out << name << "\n";

		const SampleVec &samples = m_samples[body];

		for (unsigned int i = 0 ; i < samples.size() ; i++ ) {
			const Sample sample = samples[i];
			out << "\t" << sample.timeStep;
			if (m_flags & TRACK_POSITION)
				out << "," << sample.position;
			if (m_flags & TRACK_ANGLE)
				out << "," << std::setprecision(PRINT_PRECISION) <<
					sample.angle;
			if (m_flags & TRACK_WORLD_CENTER)
				out << "," << sample.worldCenter;
			if (m_flags & TRACK_LOCAL_CENTER)
				out << "," << sample.localCenter;
			if (m_flags & TRACK_LINEAR_VELOCITY)
				out << "," << sample.linearVelocity;
			if (m_flags & TRACK_ANGULAR_VELOCITY)
				out << "," << std::setprecision(PRINT_PRECISION)
						<< sample.angularVelocity;
			out << "\n";
		}
	}
	if (!ReadBaseline())
		return false;

	return true;
}

static b2Vec2 parseVec(const std::string str)
{
	b2Vec2 ret;
	std::istringstream buf(str);
	std::string tok;
	std::getline(buf, tok, ' ');
	ret.x = (float)(atof(tok.c_str()));
	std::getline(buf, tok, ' ');
	ret.y = (float)(atof(tok.c_str()));
	return ret;
}

std::string BodyTracker::TrackFlagsToString(int32 flags) const
{
	// Create string with all the flags, or "[unknown]". Add a space at the
	// end and then delete it if necessary.  This is simply meant as a clean
	// way of doing a join of all the elements.
	std::string ret;
	if ( flags & TRACK_POSITION ) ret += "POSITION ";
	if ( flags & TRACK_ANGLE ) ret += "ANGLE ";
	if ( flags & TRACK_WORLD_CENTER ) ret += "WORLD_CENTER ";
	if ( flags & TRACK_LOCAL_CENTER ) ret += "LOCAL_CENTER ";
	if ( flags & TRACK_LINEAR_VELOCITY ) ret += "LINEAR_VELOCITY ";
	if ( flags & TRACK_ANGULAR_VELOCITY ) ret += "ANGULAR_VELOCITY ";
	if (ret.empty())
		ret = "[unknown]";
	else if ( ret[ret.length()-1] == ' ')
		ret.erase(ret.length()-1, 1);
	return ret;
}

bool BodyTracker::ReadBaseline()
{
	std::ifstream in(const_cast<const char *>(m_baselineFile.c_str()));

	// Parse the header
	std::string header;
	std::getline(in, header);
	std::istringstream buf(header);
	m_baselineFlags = 0;
	std::vector<int> flagVals;
	for (std::string tok ; std::getline(buf, tok, ' ') ; ) {
		if (tok  == "#") continue;
		else if (tok == "POSITION") flagVals.push_back(TRACK_POSITION);
		else if (tok == "ANGLE") flagVals.push_back(TRACK_ANGLE);
		else if (tok == "WORLD_CENTER") flagVals.push_back(TRACK_WORLD_CENTER);
		else if (tok == "LOCAL_CENTER") flagVals.push_back(TRACK_LOCAL_CENTER);
		else if (tok == "LINEAR_VELOCITY")
			flagVals.push_back(TRACK_LINEAR_VELOCITY);
		else if (tok == "ANGULAR_VELOCITY")
			flagVals.push_back(TRACK_ANGULAR_VELOCITY);
		else {
			m_errors.push_back("Illegal token '" + tok + "' in header");
			return false;
		}
	}

	// Create mask of all baseline flags and enforce that it matches generated
	// samples.
	m_baselineFlags = 0;
	for (size_t i = 0 ; i < flagVals.size() ; i++ )
		m_baselineFlags |= flagVals[i];
	if (m_baselineFlags != m_flags) {
		m_errors.push_back("Error: baseline data does not match generated "
						   "data");
		m_errors.push_back("\tBaseline:  " +
						   TrackFlagsToString(m_baselineFlags));
		m_errors.push_back("\tGenerated: " + TrackFlagsToString(m_flags));
		return false;
	}

	// Iterate over all strings in file.
	std::string bodyName;
	SampleVec *samples = 0;
	for (std::string str ; std::getline(in, str) ; ) {
		if (str.length() > 0 && str[0] != '\t') {
			m_baselineSamples[str] = SampleVec();
			samples = &m_baselineSamples[str];
			continue;
		}
        if (!samples) {
            continue;
        }
		Sample sample;
		std::istringstream buf(str);
		int32 field = 0;
		for ( std::string tok ; std::getline(buf, tok, ',') ; ++field ) {
			if (field == 0){
				sample.timeStep = (float)atof(tok.c_str());
				continue;
			}
			const TrackFlags flagVal = static_cast<TrackFlags>(
				flagVals[field-1]);
			switch (flagVal) {
			case TRACK_POSITION:
				sample.position = parseVec(tok);
			case TRACK_ANGLE:
				sample.angle = (float)atof(tok.c_str());
			case TRACK_WORLD_CENTER:
				sample.worldCenter = parseVec(tok);
			case TRACK_LOCAL_CENTER:
				sample.localCenter = parseVec(tok);
			case TRACK_LINEAR_VELOCITY:
				sample.linearVelocity = parseVec(tok);
			case TRACK_ANGULAR_VELOCITY:
				sample.angularVelocity = (float)atof(tok.c_str());
			}
		}
		samples->push_back(sample);
	}
	m_baselineRead = true;
	return true;
}

static bool floatsEqual(float32 a, float32 b, float32 epsilon)
{
	return fabs(a - b) < epsilon;
}

static bool vecsEqual(const b2Vec2 &a, const b2Vec2 &b, float32 epsilon)
{
	return floatsEqual(a.x, b.x, epsilon) && floatsEqual(a.y, b.y, epsilon);
}

bool BodyTracker::CompareToBaseline(const b2Body *body, int32 flag,
									float32 epsilon) const
{
	const std::string &bodyName = m_bodyNames.find(body)->second;

	std::map<const b2Body *, SampleVec>::const_iterator it =
		m_samples.find(body);
	if (it == m_samples.end()) {
		m_errors.push_back("Could not find data for body " + bodyName);
		return false;
	}
	const SampleVec &samples = it->second;

	std::map<const std::string, SampleVec>::const_iterator it2 =
			m_baselineSamples.find(bodyName);
	if (it2 == m_baselineSamples.end()) {
		m_errors.push_back("Could not find baseline data for body " +
						   bodyName);
		return false;
	}

	double printPrecision = 1.0 / pow(10.0f, PRINT_PRECISION);
	if (epsilon < printPrecision) {
		std::stringstream strstream;
		strstream << "Specified epsilon "
			<< std::setprecision(PRINT_PRECISION) << epsilon
			<< " is greater than print precision "
			<< std::setprecision(PRINT_PRECISION) <<  printPrecision << "\n";
		m_errors.push_back(strstream.str());
		return false;
	}

	// Assume that each sample in each of the two arrays represent the same
	// sample.
	std::stringstream diffs;
	const SampleVec &baselineSamples = it2->second;
	for (size_t i = 0 ; i < samples.size() ; i++ ) {
		const Sample &sample = samples[i];
		const Sample &baselineSample = baselineSamples[i];
		if ((flag & TRACK_POSITION) &&
			!vecsEqual(sample.position, baselineSample.position, epsilon))
			diffs << "\t\tsample = " << i << ", TRACK_POSITION: (" <<
				sample.position
				      << ") != (" << baselineSample.position << ")\n";
		else if ((flag & TRACK_ANGLE) &&
				 !floatsEqual(sample.angle, baselineSample.angle, epsilon))
			diffs << "\t\tsample = " << i << ", TRACK_ANGLE: " << sample.angle
				      << " != " << baselineSample.angle << "\n";
		else if ((flag & TRACK_WORLD_CENTER) &&
				 !vecsEqual(sample.worldCenter, baselineSample.worldCenter,
							epsilon))
			diffs << "\t\tsample = " << i << ", TRACK_WORLD_CENTER: (" <<
				sample.worldCenter
				      << ") != (" << baselineSample.worldCenter << ")\n";
		else if ((flag & TRACK_LOCAL_CENTER) &&
				 !vecsEqual(sample.localCenter, baselineSample.localCenter,
							epsilon))
			diffs << "\t\tsample = " << i << ", TRACK_LOCAL_CENTER: (" <<
				sample.localCenter
				      << ") != (" << baselineSample.localCenter << ")\n";
		else if ((flag & TRACK_LINEAR_VELOCITY) &&
				 !vecsEqual(sample.linearVelocity,
							baselineSample.linearVelocity, epsilon))
			diffs << "\t\tsample = " << i << ", TRACK_LINEAR_VELOCITY: (" <<
				sample.linearVelocity
				      << ") != (" << baselineSample.linearVelocity << ")\n";
		else if ((flag & TRACK_ANGULAR_VELOCITY) &&
				 !floatsEqual(sample.angularVelocity,
							  baselineSample.angularVelocity, epsilon))
			diffs << "\t\tsample = " << i << ", TRACK_ANGULAR_VELOCITY: " <<
				sample.angularVelocity << " != " <<
				baselineSample.angularVelocity << "\n";

	}
	const std::string diffsStr = diffs.str();
	if (diffsStr.empty())
		return true;

	std::stringstream strstream;
	strstream << "In '" << bodyName << "':\n";
	m_errors.push_back(strstream.str() + diffsStr);
	return false;
}

const std::vector<std::string> &BodyTracker::GetErrors() const
{
	// If an error occurred, make sure you print out a line stating the
	// filenames.
	if (!m_errors.empty()) {
		m_errors.insert(m_errors.begin(), "Mismatch with baseline '" +
						m_baselineFile + "':");
	}
	return m_errors;
}

// Set the root directory for baseline and output files using argv[0]
// (the program's path).
void BodyTracker::SetWorkingDirectory(const char *argv0)
{
	std::string executablePath = argv0;
	std::string separator = executablePath.rfind('/') != std::string::npos ?
		"/" : "\\";
	s_baselineRootDir = executablePath.substr(
		0, executablePath.rfind(separator)) + separator;
#if defined(_WIN32) || defined(__APPLE__)
	// MSVC and Xcode put test executables in a subdirectory of the project so
	// change the baseline directory to the container directory.
	s_baselineRootDir = s_baselineRootDir + ".." + separator;
#endif // defined(_WIN32) || defined(__APPLE__)
}
