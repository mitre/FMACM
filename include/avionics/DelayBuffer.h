// ****************************************************************************
// NOTICE
//
// This work was produced for the U.S. Government under Contract 693KA8-22-C-00001
// and is subject to Federal Aviation Administration Acquisition Management System
// Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV (Oct. 1996).
//
// The contents of this document reflect the views of the author and The MITRE
// Corporation and do not necessarily reflect the views of the Federal Aviation
// Administration (FAA) or the Department of Transportation (DOT). Neither the FAA
// nor the DOT makes any warranty or guarantee, expressed or implied, concerning
// the content or accuracy of these views.
//
// For further information, please contact The MITRE Corporation, Contracts Management
// Office, 7515 Colshire Drive, McLean, VA 22102-7539, (703) 983-6000.
//
// 2023 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#pragma once

#include <queue>

template <class T>
class DelayBuffer {
  public:
   DelayBuffer(int latency_in) {
      // clear the queue
      while (!this->mQueue.empty()) {
         this->mQueue.pop();
      }
      this->latency = latency_in;
   }

   DelayBuffer() {}

   ~DelayBuffer(void) {}

   void set_latency(int latency_in) { this->latency = latency_in; }

   int get_size() { return mQueue.size(); }

   T back() { return mQueue.back(); }

   T push_pop(T in) {
      if (this->latency == 0)  // zero latency:
      {
         return in;
      }
      // else:
      T out;
      if ((int)mQueue.size() < this->latency) {
         mQueue.push(in);
         out = mQueue.front();
      } else {
         out = mQueue.front();
         mQueue.pop();
         mQueue.push(in);
      }

      return out;
   }

  private:
   // Data:
   int latency;
   std::queue<T> mQueue;
};
