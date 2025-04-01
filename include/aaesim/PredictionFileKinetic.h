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

#include "aaesim/AircraftEntity.h"
#include "public/PredictionFileBase.h"
#include "aaesim/AircraftDataIterationWriter.h"
#include "aaesim/KineticPredictionDataIterationWriter.h"

namespace aaesim {
class PredictionFileKinetic final : public AircraftDataIterationWriter,
                                    public KineticPredictionDataIterationWriter,
                                    private PredictionFileBase {
  public:
   PredictionFileKinetic()
      : aaesim::AircraftDataIterationWriter("", "_predicted_kinetic_trajectory.csv"), fms_prediction_data() {}
   void Finish() override;
   std::string GetFilename() const override { return GetOutputFilename(); }
   void Gather(const int &iteration_number, const aaesim::AircraftEntity &aircraft) override;
   void Gather(const int &iteration_number, const aaesim::KineticPredictionEntity &entity) override;

  private:
   static log4cplus::Logger logger;
   static std::vector<std::string> COLUMN_NAMES;
   inline static std::shared_ptr<PredictionFileKinetic> m_instance = nullptr;

   void StoreKineticTrajectoryData(const int iteration, const std::string &acid,
                                   const FlightManagementSystem &flight_management_system);
   std::map<std::string, std::vector<PredictionData>> fms_prediction_data;
};
}  // namespace aaesim