# Copyright (C) 2009 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# Test for gtest. Run using 'runtest'.
# The linux build and tests are run under valgrind by 'runtest'.

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

# TODO: Refactor these as 1st class build templates as suggested in
# review of the original import.

libgtest_test_common_includes := \
    $(LOCAL_PATH)/../include \
    $(LOCAL_PATH)/..

libgtest_test_includes := $(libgtest_test_common_includes) bionic external/stlport/stlport
libgtest_test_static_lib := libgtest_main libgtest
libgtest_test_shared_lib := libstlport

libgtest_test_host_includes := $(libgtest_test_common_includes)
libgtest_test_host_static_lib := libgtest_main_host libgtest_host
libgtest_test_host_shared_lib :=

# $(2) and $(4) must be set or cleared in sync. $(2) is used to
# generate the right make target (host vs device). $(4) is used in the
# module's name and to have different module names for the host vs
# device builds. Finally $(4) is used to pick up the right set of
# libraries, typically the host libs have a _host suffix in their
# names.
# $(1): source list
# $(2): "HOST_" or empty
# $(3): extra CFLAGS or empty
# $(4): "_host" or empty
# $(5): "TARGET_OUT_DATA_NATIVE_TESTS" or empty (where to install)
define _define-test
$(foreach file,$(1), \
  $(eval include $(CLEAR_VARS)) \
  $(eval LOCAL_CPP_EXTENSION := .cc) \
  $(eval LOCAL_SRC_FILES := $(file)) \
  $(eval LOCAL_C_INCLUDES := $(libgtest_test$(4)_includes)) \
  $(eval LOCAL_MODULE := $(notdir $(file:%.cc=%))$(4)) \
  $(eval LOCAL_CFLAGS += $(3)) \
  $(eval LOCAL_STATIC_LIBRARIES := $(libgtest_test$(4)_static_lib)) \
  $(eval LOCAL_SHARED_LIBRARIES := $(libgtest_test$(4)_shared_lib)) \
  $(if $(2),,$(eval LOCAL_MODULE_TAGS := tests)) \
  $(eval LOCAL_MODULE_PATH := $($(5))) \
  $(eval include $(BUILD_$(2)EXECUTABLE)) \
)
endef

define host-test
$(call _define-test,$(1),HOST_,-O0,_host,)
endef

define target-test
$(call _define-test,$(1),,,,TARGET_OUT_DATA_NATIVE_TESTS)
endef

sources := \
  gtest-death-test_test.cc \
  gtest-filepath_test.cc \
  gtest-linked_ptr_test.cc \
  gtest-message_test.cc \
  gtest-options_test.cc \
  gtest-port_test.cc \
  gtest_environment_test.cc \
  gtest_no_test_unittest.cc \
  gtest_pred_impl_unittest.cc \
  gtest_repeat_test.cc \
  gtest-test-part_test.cc \
  gtest-typed-test_test.cc \
  gtest-typed-test2_test.cc \
  gtest_stress_test.cc \
  gtest_unittest.cc \
  gtest_prod_test.cc

ifeq ($(NDK_PROJECT_PATH),)
$(call host-test, $(sources))
endif
$(call target-test, $(sources))
