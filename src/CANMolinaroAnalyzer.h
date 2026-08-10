#ifndef CANMOLINARO_ANALYZER_H
#define CANMOLINARO_ANALYZER_H

//----------------------------------------------------------------------------------------

#include <Analyzer.h>
#include "CANMolinaroAnalyzerResults.h"
#include "CANMolinaroSimulationDataGenerator.h"

//----------------------------------------------------------------------------------------

class CANMolinaroAnalyzerSettings;

//----------------------------------------------------------------------------------------

class ANALYZER_EXPORT CANMolinaroAnalyzer : public Analyzer2 {

  public: CANMolinaroAnalyzer();

  public: virtual ~CANMolinaroAnalyzer();

  public: virtual void SetupResults();

  public: virtual void WorkerThread();

  public: virtual U32 GenerateSimulationData (U64 newest_sample_requested,
                                              U32 sample_rate,
                                              SimulationChannelDescriptor** simulation_channels);
  public: virtual U32 GetMinimumSampleRateHz();

  public: virtual const char* GetAnalyzerName() const;

  public: virtual bool NeedsRerun();

  private: void enterBit (const bool inBit, const U64 inSampleNumber) ;
  private: void decodeFrameBit (const bool inBit, const U64 inSampleNumber) ;


//--- Protected properties
  protected: std::unique_ptr < CANMolinaroAnalyzerSettings > mSettings;
  protected: std::unique_ptr < CANMolinaroAnalyzerResults > mResults;

  protected: CANMolinaroSimulationDataGenerator mSimulationDataGenerator;
  protected: bool mSimulationInitilized ;
  protected: bool mAcked ;

  //Serial analysis vars:
  protected: U32 mSampleRateHz;
  protected: U32 mStartOfStopBitOffset;
  protected: U32 mEndOfStopBitOffset;

  public: inline U32 sampleRateHz (void) const { return mSampleRateHz ;  }
  public: U32 bitRate (void) const ;

//---------------- CAN decoder properties
//--- CAN protocol
  private: typedef enum  {
    IDLE, IDENTIFIER, CONTROL, EXTENDED_IDF, DATA,
    CRC15, CRC_DEL, ACK, END_OF_FRAME, INTERMISSION, DECODER_ERROR
  } FrameFieldEngineState ;

  private: FrameFieldEngineState mFrameFieldEngineState ;
  private: int mFieldBitIndex ;
  private: int mConsecutiveBitCountOfSamePolarity ;
  private: bool mPreviousBit ;
  private: bool mUnstuffingActive ;

  private: U64 mStartOfFrameSampleNumber ;
  private: U64 mStuffBitCount ;
  private: U64 mStartOfFieldSampleNumber ;

//--- The RTR bit's own sample boundaries, captured when it's processed so
//    it can be split into its own bubble later, when the identifier bubble
//    it used to be lumped into is finally closed (see handle_IDENTIFIER_state
//    and handle_EXTENDED_IDF_state).
  private: U64 mRtrStartSampleNumber ;
  private: U64 mRtrEndSampleNumber ;

//--- SRR's own boundaries (extended frames only). SRR shares the same bit
//    slot RTR uses on the standard path, captured into mRtrStart/EndSample-
//    Number at bit 12 in handle_IDENTIFIER_state, then copied out here
//    before that bit-12 capture gets overwritten by the *real* RTR bit
//    later on (bit 19, in handle_EXTENDED_IDF_state). Held onto until R1,
//    where the full 29-bit identifier is finally known, so the identifier
//    bubble can be split into two pieces around SRR with the same
//    (complete, correct) value shown in both.
  private: U64 mSrrStartSampleNumber ;
  private: U64 mSrrEndSampleNumber ;

//--- Received frame
  private: typedef enum {dataFrame, remoteFrame} FrameType ;
  private: uint32_t mIdentifier ;
  private: FrameType mFrameType ; // data, remote
  private: bool mExtended ;
  private: int mDataCodeLength ;
  private: uint8_t mData [8] ;
  private: U16 mCRC15Accumulator ;
  private: U16 mCRC15 ;

//--- Tracks how far the current frame got, for the consolidated FrameV2 row
//    (a frame that errors out partway through still gets one row, with
//    whichever fields were actually captured before the error).
  private: bool mHaveIdentifier ;
  private: bool mHaveCrc ;
  private: bool mHaveAck ;
  private: CanErrorReason mErrorReason ;

//---------------- CAN decoder methods
  private: void enterBitInCRC15 (const bool inBit) ;
  private: void addMark (const U64 inSampleNumber, const AnalyzerResults::MarkerType inMarker) ;
  private: void addBubble (const U8 inBubbleType, const U64 inData1, const U64 inData2, const U64 inEndSampleNumber) ;
  private: void emitConsolidatedFrameV2 (const U64 inEndSampleNumber, const bool inError) ;
  private: void enterInErrorMode (const U64 inSampleNumber, const CanErrorReason inReason) ;

  private: void handle_IDLE_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_IDENTIFIER_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_EXTENDED_IDF_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CONTROL_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_DATA_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CRC15_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_CRCDEL_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_ACK_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_ENDOFFRAME_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_INTERMISSION_state (const bool inBit, const U64 inSampleNumber) ;
  private: void handle_DECODER_ERROR_state (const bool inBit, const U64 inSampleNumber) ;
} ;

//----------------------------------------------------------------------------------------

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

//----------------------------------------------------------------------------------------

#endif //CANMOLINARO_ANALYZER_H
