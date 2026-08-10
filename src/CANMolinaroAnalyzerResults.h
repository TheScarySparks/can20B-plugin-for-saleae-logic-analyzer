#ifndef CANMOLINARO_ANALYZER_RESULTS
#define CANMOLINARO_ANALYZER_RESULTS

//----------------------------------------------------------------------------------------

#include <AnalyzerResults.h>

//----------------------------------------------------------------------------------------

enum CanFrameType {
  STANDARD_IDENTIFIER_FIELD_RESULT,
  EXTENDED_IDENTIFIER_FIELD_RESULT,
  SRR_FIELD_RESULT,
  RTR_FIELD_RESULT,
  IDE_FIELD_RESULT,
  R0_FIELD_RESULT,
  R1_FIELD_RESULT,
  CONTROL_FIELD_RESULT,
  DATA_FIELD_RESULT,
  CRC_FIELD_RESULT,
  CRC_DEL_FIELD_RESULT,
  ACK_FIELD_RESULT,
  ACK_DEL_FIELD_RESULT,
  EOF_FIELD_RESULT,
  INTERMISSION_FIELD_RESULT,
  CAN_ERROR_RESULT
} ;

//----------------------------------------------------------------------------------------
//  Which specific check failed when a frame enters the error state. Stored
//  as a small int code (Frame.mData1) since GenerateText is only ever
//  handed the Frame object, not live analyzer state.
//----------------------------------------------------------------------------------------

enum CanErrorReason {
  ERROR_STUFF,
  ERROR_FORM_R0,
  ERROR_FORM_R1,
  ERROR_FORM_CRC_DEL,
  ERROR_FORM_ACK_DEL,
  ERROR_FORM_EOF,
  ERROR_FORM_INTERMISSION,
  ERROR_CRC
} ;

const char* CanErrorReasonText (CanErrorReason inReason) ;

//----------------------------------------------------------------------------------------

class CANMolinaroAnalyzer;
class CANMolinaroAnalyzerSettings;

//----------------------------------------------------------------------------------------

class CANMolinaroAnalyzerResults : public AnalyzerResults {
public:
  CANMolinaroAnalyzerResults( CANMolinaroAnalyzer* analyzer, CANMolinaroAnalyzerSettings* settings );
  virtual ~CANMolinaroAnalyzerResults();

  virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
  virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

  virtual void GenerateFrameTabularText(U64 frame_index, DisplayBase display_base );
  virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
  virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

protected: //functions
  void GenerateText (const Frame & inFrame,
                     const DisplayBase inDisplayBase,
                     const bool inBubbleText,
                     std::stringstream & ioText) ;

protected:  //vars
  CANMolinaroAnalyzerSettings* mSettings;
  CANMolinaroAnalyzer* mAnalyzer;
};

//----------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_RESULTS
