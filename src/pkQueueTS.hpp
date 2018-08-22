/**************************************************************************************
*
* IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
*
* By downloading, copying, installing or using the software you agree to this license.
* If you do not agree to this license, do not download, install,
* copy or use the software.
*
*                           License Agreement
*                           For pkQueueTS
*
*  pkQueueTS - http://pklab.net/index.php?id=395
*  Copyright (C) 2016 PkLab.net all rights reserved.
*
*  Third party copyrights are property of their respective owners.
*
*  Redistribution and use in source and binary forms, with or without modification,
*  are permitted provided that the following conditions are met:
*
*  1. Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*  3. The name of the author may not be used to endorse or promote products
*     derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
* EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************************/

#ifndef PKQUEUETS_H
#define PKQUEUETS_H

#if __cplusplus < 199711L
#error "This file requires C++ 11 !"
#endif

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace  cv;

//! \defgroup  pkQueueTS A Thread safe queue by PkLab
//! @{

/** \brief Thread Safe Queue result codes
 */
enum pkQueueResults
{
    PK_QTS_OK,          //!< The result is correct
    PK_QTS_TIMEOUT,     //!< A time-out has reached while push or pop
    PK_QTS_EMPTY,       //!< The queue is empty and we wont wait
};

/** \brief Simple structure for an OpenCV Mat with deep copy constructor and assignment operator.
*
* This class encapsulates a `cv::Mat` and provides custom assignment operator and
* copy constructor to perform deep copy of matrix data too.
*
* This is because the `cv::Mat` assignment operator and the copy constructor
* just copy the Mat header but not the matrix data.
*
* This class is useful for Mats buffering. For example:
* - `std::queue::push(cv::Mat)` just creates a copy of `cv::Mat` header in the queue.
* - `std::queue::push(MatCapsule)` creates a deep copy of `cv::Mat` in the queue.
*
* \tparam TAdditionalStorage Here you can attach your own data type to hold any kind of information.
* In case your data type contains simple types, like a structure, you can forget about copying.
* If your data type holds any pointers or complex data type, you have to take care
* its copy constructor and assignment operator.
*
* \see
* - Assignment operator [cv::Mat::operator=](http://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html#aed1f81fe7efaacc2bd95149cdfa34302)
* - Copy constructor [cv::Mat::Mat(const Mat &m)](http://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html#a294eaf8a95d2f9c7be19ff594d06278e)
*
*/
template <class TAdditionalStorage = int>
class MatCapsule_
{
public:
    //! The OpenCV Mat
    Mat mat;

    /** \brief Storage for additional information (Template)
    *
    * You can define you own data type to hold any kind of information.
    * - In case your `TAdditionalStorage` holds just simple types you can forget about copying.
    * - In case `TAdditionalStorage` holds any pointers or complex data type, you have to take care
    *   of copy constructor and assignment operator.
    */
    TAdditionalStorage vars;

    /** \brief Default constructor
    */
    MatCapsule_() { }

    /** \brief Class destructor. Called by queue::pop
    */
    ~MatCapsule_() { }

    /** \brief The copy constructor. Called by queue::push
    *
    * Performs a deep copy of the `mat` member and its data.
    * Calls the `TAdditionalStorage` copy constructor if any.
    */
    MatCapsule_(const MatCapsule_& src)
    {
        vars = TAdditionalStorage(src.vars); //Calls the copy constructor for the data type
        src.mat.copyTo(mat);
    }

    /** \brief The assignment operator. Called BY queue::front
     * \warning In case you have any member pointers you need to clone them.
     * This is because, next calls to queue::pop calls the class destructor destroying all data.
     * With `cv::Mat` the matrix data is still valid thanks to its internal memory counter.
     */
    MatCapsule_& operator=(const MatCapsule_& src)
    {
        //Sanity check
        if (this == &src) return *this;

        vars = src.vars;    // Calls assignment operator for the data type
        mat = src.mat;      // Just copy the Mat header

        return *this;
    }
};

/** \brief For your convenience, If you don't need any additional storage */
typedef MatCapsule_<int> MatCapsule;

/** \brief Abstract base class for handling custom pkQueueTS::Push event.
 *
 * It provides an abstract interface to create custom OnPush event handler.
 * The OnPush event could be used to perform any custom actions like memory analysis on the pkQueueTS.
 *
 * ### How to use the OnPush event
 * Because this class is pure virtual, you have to derive your own class and write your own OnPush().
 * The "OnPush" constructor pkQueueTS::pkQueueTS(pkQueueOnPushBase* onPushHandler) should
 * be used
 *
 * In this case, pkQueueTS::Push() calls `YourDerivedClass::OnPush()` within its own lock.
 */
class pkQueueOnPushBase
{
public:
    /** \brief OnPush event handler interface
     *
     *  User has to defines its own class and implements the method to take needed actions.
     *
     * \param queueSize Queue size after current push.
     * \param elementPtr Pointer to the last pushed element in the queue <`&m_queue.back()`>
     * \note This is called by pkQueueTS::Push() within a proper lock on the queue.
     * Therefore the OnPush() method is thread safe vs those threads that are writing /reading
     * the queue, also in case of multiple producer/consumer.
     * \warning Do not use `elementPtr` out of here because pointed to object will be destroyed
     * on next pkQueueTS::Pop().
     */
    virtual void OnPush(size_t queueSize, const void *elementPtr) = 0;
};

///////////////////////////////////////////////////////////////////////////////
/** \brief Thread Safe Queue
*
* \author
* pkQueueTS - http://pklab.net/index.php?id=395
* Copyright (C) 2016 PkLab.net all rights reserved.
*
* This is a thread safe queue, multiple producer multiple consumer.
* It's based on [std::queue](http://en.cppreference.com/w/cpp/container/queue) with single lock.
* A [condition_variable](http://en.cppreference.com/w/cpp/thread/condition_variable)
* with time-out is used to wait for new data.
*
* ## A thread safe queue for OpenCV Mats
*
* This class can be used to manage queue of [OpenCV Mats](http://docs.opencv.org/master/d3/d63/classcv_1_1Mat.html).
* Thanks to [Automatic Memory Management](http://docs.opencv.org/master/d1/dfb/intro.html) of `cv::Mat`
* and C++/STL memory recycling, use of `std::queue` with `cv::Mat` is memory effective.
*
* Unfortunately `cv::Mat` can't be used directly as element because its assignment operator and copy constructor
* just copy the Mat header but not the matrix data. A simple solution is to encapsulates `cv::Mat`
* into a structure or class and to write adequate assignment and constructor. The template class
* MatCapsule_ (or its simple alias `MatCapsule` ) is provided here to this purpose.
*
* Definitely you can write:
*
* \snippet pkQueueTS_TestSimple.cpp pkQueueTS_BasicUsage
*
* ## Additional storage
*
* The template class MatCapsule_ allows to attach any kind of additional storage to the capsule.
* For example you can have:
*
* \snippet pkQueueTS_TestSimple.cpp pkQueueTS_Additional_storage
*
* ## Memory statistics
*
* You can easily measure memory usage and allocations performed while using this class.
* Just write your own `OnPush` event handler and collect wanted measurement (see pkQueueOnPushBase).
*
* For example you can collect all addresses of pushed elements using a simple `std::map` as below:
*
* \snippet pkQueueTS_TestSimple.cpp pkQueueTS_Stats
*
* Please see http://pklab.net/?&id=396 for a detailed example and analysis
*/
template <class TElement>
class pkQueueTS
{
public:

    /** \brief Default constructor
    */
    pkQueueTS():m_onPushHandler(nullptr) {  }

    /** \overload
     *  \brief "OnPush" Constructor
     *
     * If the "OnPush" constructor is used, a custom object will be invoked each time
     * pkQueueTS::Push() is called. In other words you have an OnPush event handler
     * that can be used to perform any custom actions on the queue.
     *
     * \param onPushHandler A pointer to custom object be derived from
     *                      the abstract class pkQueueOnPushBase
     */
    pkQueueTS(pkQueueOnPushBase* onPushHandler)
    {
        m_onPushHandler = onPushHandler;
    }

    /** \brief Default destructor
    *
    * Calls element destructor for all elements in the queue (if any).
    * \warning If element is a pointer, the pointed to objects are not destroyed.
    */
    ~pkQueueTS()
    {
        lock_guard<mutex> lock(m_mutex);
        while (!m_queue.empty())
            m_queue.pop();  //if you are here than you are discarding some elements
    }

    /** \brief Retrieves the oldest element in the queue, if any or if wait.
    *
    * Blocks the caller thread until a new element is available or after the specified
    * time-out duration. When/If the element has been retrieved correctly is it's removed
    * from the queue.
    *
    * \param [out] element If PK_QTS_OK is returned, contains the oldest element in the queue.
    * \param timeoutMs You can specify a time (milliseconds) to wait for a valid elements
    *        - `timeoutMs = 0` [default] means no wait at all
    *        - `timeoutMs > 0` means wait for given time out
    * \return pkQueueResults One of the following:
    *         - PK_QTS_OK The element has been retrieved correctly
    *         - PK_QTS_TIMEOUT  The queue is empty but we wont wait
    *         - PK_QTS_EMPTY Time out has reached waiting a valid element
    */
    pkQueueResults Pop(TElement &element, unsigned timeoutMs = 0)
    {
        unique_lock<mutex> lock(m_mutex);
        // wait for a data (ignoring spurious awakenings)
        while (m_queue.empty())
        {
            // if no data and won't wait;
            if (timeoutMs == 0)
            {
                return PK_QTS_EMPTY;
            }
            // else wait data with time-out
            else if (cv_status::timeout == m_dataReady.wait_for(lock, chrono::milliseconds(timeoutMs)))
            {
                // Time out while waiting a valid element
                return PK_QTS_TIMEOUT;
            }
        } // data available
        element = m_queue.front(); //calls TElement::=operator
        m_queue.pop();
        return PK_QTS_OK;
    }

    /** \brief Inserts an element at the end of the queue
    *
    * \param element The element to add.
    * \return The queue size after that the element has been added
    */
    size_t Push(const TElement &element)
    {
        unique_lock<mutex> lock(m_mutex);
        m_queue.push(element);  //calls TElement::copy_constructor

        size_t N = m_queue.size();
        if (m_onPushHandler)
        {
            void * ptr = &(m_queue.back());
            m_onPushHandler->OnPush(N, ptr);
        }

        // The notifying thread does not need to hold the lock on the
        // same mutex as the one held by the waiting thread(s);
        // in fact doing so is a pessimization
        lock.unlock();
        m_dataReady.notify_one();
        return N;
    }

    /** \brief Returns the size of the queue
    */
    size_t Size()
    {
        lock_guard<mutex> lock(m_mutex);
        return m_queue.size();
    }

    /** \brief Test whether queue is empty
    */
    bool Empty() const
    {
        lock_guard<mutex> lock(m_mutex);
        return m_queue.empty();
    }

private:
    std::queue<TElement>    m_queue;
    std::condition_variable m_dataReady;
    mutable std::mutex      m_mutex;
    pkQueueOnPushBase*      m_onPushHandler;

};

//! @}
#endif //PKQUEUETS_H
