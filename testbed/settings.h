// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

struct Settings
{
	Settings()
	{
		Reset();
	}

	void Reset()
	{
		m_testIndex = 0;
		m_windowWidth = 1600;
		m_windowHeight = 900;
		m_hertz = 60.0f;
		m_velocityIterations = 8;
		m_positionIterations = 3;
		m_drawShapes = true;
		m_drawJoints = true;
		m_drawAABBs = false;
		m_drawContactPoints = false;
		m_drawContactNormals = false;
		m_drawContactImpulse = false;
		m_drawFrictionImpulse = false;
		m_drawCOMs = false;
		m_drawStats = false;
		m_drawProfile = false;
		m_enableWarmStarting = true;
		m_enableContinuous = true;
		m_enableSubStepping = false;
		m_enableSleep = true;
		m_pause = false;
		m_singleStep = false;
	}

	void Save();
	void Load();

	int m_testIndex;
	int m_windowWidth;
	int m_windowHeight;
	float m_hertz;
	int m_velocityIterations;
	int m_positionIterations;
	bool m_drawShapes;
	bool m_drawJoints;
	bool m_drawAABBs;
	bool m_drawContactPoints;
	bool m_drawContactNormals;
	bool m_drawContactImpulse;
	bool m_drawFrictionImpulse;
	bool m_drawCOMs;
	bool m_drawStats;
	bool m_drawProfile;
	bool m_enableWarmStarting;
	bool m_enableContinuous;
	bool m_enableSubStepping;
	bool m_enableSleep;
	bool m_pause;
	bool m_singleStep;
};
