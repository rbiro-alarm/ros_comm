/* adc_condition.h -*- c++ -*-
 *
 * Condition variable template with
 * callbacks so we can chain them
 * and have one wake the next.
 * that way we can get ROS to
 * stop wasting CPU time and
 * still do things sanely.
 */

#ifndef ADC_CONDITION_H
#define ADC_CONDITION_H

#include <boost/container/list.hpp>
#include <boost/function.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/bind.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

template <typename T> class ADCCondition
{
protected:
  T value;
  boost::mutex mutex;
  boost::condition cond;

  boost::container::list<boost::function<void(T, T)> > callbacks;

  void doCallbacks(T ov, T nv) {
    typedef boost::function<void(T, T)> func;
    BOOST_FOREACH(func f, callbacks) {
      f(ov, nv);
    }
  }

  template <typename S> void myCallback(S ov, S nv) {
    // Wake myself up so I can check the
    // other value.
    cond.notify_all();
  }

public:
  ADCCondition() { }
  ADCCondition(const T iv) {
    value = iv;
  }

  virtual ~ADCCondition() { }

  operator T() {
    boost::interprocess::scoped_lock<boost::mutex> lock(mutex);
    return value;
  }

  T operator =(T nv) {
    T ov;
    {
      boost::interprocess::scoped_lock<boost::mutex> lock(mutex);
      ov = value;
      value = nv;
    }
    // Do this outside the lock so I have much
    // less of a chance of deadlocking.
    if (ov != nv) {
      doCallbacks(ov, nv);
    }
    cond.notify_all();
    return value;
  }

  void registerCallback(boost::function<void(T, T)> f) {
    callbacks.push_back(f);
  }

  void deregisterCallback(boost::function<void(T, T)> f) {
    callbacks.remove(f);
  }

  void wait() {
      boost::mutex::scoped_lock lock(mutex);
      cond.wait(lock);
  }

  template <typename S> void addParent(ADCCondition<S> &parent) {
    typedef void (ADCCondition<T>::*mf_type)(S, S);
    void (ADCCondition<T>::*mf)(S, S) = &ADCCondition<T>::myCallback;
    boost::function<void(S, S)> f =
      boost::bind(static_cast<mf_type>(&ADCCondition<T>::myCallback), this, _1, _2);
    parent.registerCallback(f);
  }
 
};

#endif /* ADC_CONDITION_H_ */
