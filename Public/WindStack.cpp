// ****************************************************************************
// NOTICE
//
// This is the copyright work of The MITRE Corporation, and was produced
// for the U. S. Government under Contract Number DTFAWA-10-C-00080, and
// is subject to Federal Aviation Administration Acquisition Management
// System Clause 3.5-13, Rights In Data-General, Alt. III and Alt. IV
// (Oct. 1996).  No other use other than that granted to the U. S.
// Government, or to those acting on behalf of the U. S. Government,
// under that Clause is authorized without the express written
// permission of The MITRE Corporation. For further information, please
// contact The MITRE Corporation, Contracts Office, 7515 Colshire Drive,
// McLean, VA  22102-7539, (703) 983-6000. 
//
// Copyright 2018 The MITRE Corporation. All Rights Reserved.
// ****************************************************************************

#include "public/WindStack.h"

WindStack::WindStack() {
   // TODO Auto-generated constructor stub

}

WindStack::WindStack(const int min,
                     const int max) {
   setBounds(min, max);
}

WindStack::~WindStack() {
   // TODO Auto-generated destructor stub
}

/*
This was useful for migration, but we don't need it any more.

WindStack::WindStack(const DMatrix& dMatrix) {
	load(dMatrix);
}

void WindStack::load(const DMatrix& dMatrix) {
	int altCol = dMatrix.get_min_colomn();
	int speedCol = dMatrix.get_max_colomn();
	int min = dMatrix.get_min_row();
	int max = dMatrix.get_max_row();
	setBounds(min, max);
	for (int i = min; i <= max; i++) {
		set(i, Units::FeetLength(dMatrix.get(i, altCol)), Units::KnotsSpeed(dMatrix.get(i, speedCol)));
	}
}*/

bool WindStack::operator==(const WindStack &obj) const {

   // Generic equals operator.
   //
   // obj:comparison object.
   // returns true if obj matches.
   //         false if obj doesn't match


   bool match = ((this->get_min_row() == obj.get_min_row()) &&
                 (this->get_max_row() == obj.get_max_row()));

   for (auto ix = this->get_min_row(); (match && (ix <= this->get_max_row())); ix++) {
      match = match && (this->getAltitude(ix) == obj.getAltitude(ix)) &&
              (this->getSpeed(ix) == obj.getSpeed(ix));
   }

   return match;

}

bool WindStack::operator!=(const WindStack &obj) const {

   // Generic not equals operator.
   //
   // obj:comparison object.
   // returns true if obj doesn't match.
   //         false if obj matches.

   return !this->operator==(obj);

}

UVector<Units::FeetLength> WindStack::getAltitudeVector() {
   return altitude;
}

UVector<Units::KnotsSpeed> WindStack::getSpeedVector() {
   return speed;
}

Units::FeetLength WindStack::getAltitude(const int index) const {
   return altitude.get(index);
}

Units::KnotsSpeed WindStack::getSpeed(const int index) const {
   return speed.get(index);
}

void WindStack::setBounds(int min,
                          int max) {
   altitude.setBounds(min, max);
   speed.setBounds(min, max);
}

int WindStack::get_min_row() const {
   return altitude.get_min();
}

int WindStack::get_max_row() const {
   return altitude.get_max();
}

void WindStack::set(const int index,
                    const Units::Length a,
                    const Units::Speed s) {
   altitude.set(index, a);
   speed.set(index, s);
}

void WindStack::ascendSort() {
   //std::sort(&rows[0],&rows[maxRow-minRow+1],&DMatrix::rowsComparator);
   // std::sort(&altitude,&speed);
   // Because std::sort doesn't seem to apply to Units, implement our own.
   // bubble sort
   bool dirty = true;
   for (int top = altitude.get_max(); dirty && (top > altitude.get_min()); top--) {
      dirty = false;
      for (int i = altitude.get_min(); i < top; i++) {
         if (altitude[i] > altitude[i + 1]) {
            dirty = true;
            Units::Length tempA = altitude[i];
            Units::Speed tempS = speed[i];
            altitude[i] = altitude[i + 1];
            speed[i] = speed[i + 1];
            altitude[i + 1] = tempA;
            speed[i + 1] = tempS;
         }
      }
   }
}
