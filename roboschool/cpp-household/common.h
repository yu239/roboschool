#pragma once

#ifndef BOOST_SMART_POINTER
#include <memory>
namespace smart_pointer = std;
#else
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
namespace smart_pointer = boost;
#endif