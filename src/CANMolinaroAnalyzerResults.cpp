#include "CANMolinaroAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "CANMolinaroAnalyzer.h"
#include "CANMolinaroAnalyzerSettings.h"
#include <iostream>
#include <fstream>
#include <sstream>

//----------------------------------------------------------------------------------------

const char* CanErrorReasonText (CanErrorReason inReason) {
  switch (inReason) {
  case ERROR_STUFF : return "Stuff Error" ;
  case ERROR_FORM_R0 : return "Form Error (R0)" ;
  case ERROR_FORM_R1 : return "Form Error (R1)" ;
  case ERROR_FORM_CRC_DEL : return "Form Error (CRC Delimiter)" ;
  case ERROR_FORM_ACK_DEL : return "Form Error (ACK Delimiter)" ;
  case ERROR_FORM_EOF : return "Form Error (EOF)" ;
  case ERROR_FORM_INTERMISSION : return "Form Error (Intermission)" ;
  case ERROR_CRC : return "CRC Error" ;
  default : return "Error" ;
  }
}

//----------------------------------------------------------------------------------------

CANMolinaroAnalyzerResults::CANMolinaroAnalyzerResults (CANMolinaroAnalyzer* analyzer,
                                                        CANMolinaroAnalyzerSettings* settings) :
AnalyzerResults (),
mSettings (settings),
mAnalyzer (analyzer) {
}

//----------------------------------------------------------------------------------------

CANMolinaroAnalyzerResults::~CANMolinaroAnalyzerResults (void) {
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateText (const Frame & inFrame,
                                               const DisplayBase inDisplayBase,
                                               const bool inBubbleText,
                                               std::stringstream & ioText) {
  const U32 sampleRateHz = mAnalyzer->sampleRateHz () ;
  const U32 bitRate = mAnalyzer->bitRate () ;
  const U32 samplesPerBit = sampleRateHz / bitRate ;
//  const U64 triggerSample = mAnalyzer->GetTriggerSample () ;
  char numberString [128] = "" ;
  switch (inFrame.mType) {
  case STANDARD_IDENTIFIER_FIELD_RESULT :
//     AnalyzerHelpers::GetTimeString (inFrame.mStartingSampleInclusive, triggerSample, sampleRateHz, numberString, 128) ;
//     ioText << numberString ;
//    ioText << (((triggerSample) * 1000000) / sampleRateHz) << " µs, " ;
//     ioText << (((inFrame.mStartingSampleInclusive - triggerSample) * 1000000) / sampleRateHz) << " µs: " ;
//     ioText << (((inFrame.mEndingSampleInclusive - triggerSample) * 1000000) / sampleRateHz) << " µs: " ;

    // AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 12, numberString, 128);
    snprintf (numberString, 128, "0x%03llX", inFrame.mData1) ;
    ioText << "STD ID: " ;
    ioText << numberString ;
    ioText << "\n" ;
    break ;
  case EXTENDED_IDENTIFIER_FIELD_RESULT :
    // AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 32, numberString, 128);
    snprintf (numberString, 128, "0x%08llX", inFrame.mData1) ;
//    ioText << (((inFrame.mStartingSampleInclusive - triggerSample) * 1000000) / sampleRateHz) << " µs: " ;
    ioText << "EXT ID: " ;
    ioText << numberString ;
    ioText << "\n" ;
    break ;
  case SRR_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "SRR\n" ;
    }
    break ;
  case RTR_FIELD_RESULT :
    if (inBubbleText) {
      ioText << ((inFrame.mData1 == 0) ? "RTR: True\n" : "RTR: False\n") ;
    }
    break ;
  case IDE_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "IDE\n" ;
    }
    break ;
  case R0_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "R0\n" ;
    }
    break ;
  case R1_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "R1\n" ;
    }
    break ;
  case CONTROL_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "DLC: " << inFrame.mData1 << "\n" ;
    }
    break ;
  case DATA_FIELD_RESULT :
//    AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 8, numberString, 128);
    snprintf (numberString, 128, "0x%02llX", inFrame.mData1) ;
    if (!inBubbleText) {
      ioText << "  " ;
    }
    ioText << "D" << inFrame.mData2 << ": " << numberString << "\n" ;
    break ;
  case CRC_FIELD_RESULT : // Data1: CRC, Data2: is 0 if CRC ok
    // AnalyzerHelpers::GetNumberString (inFrame.mData1, inDisplayBase, 16, numberString, 128);
    snprintf (numberString, 128, "0x%04llX", inFrame.mData1) ;
    if (inFrame.mData2 != 0) {
      if (!inBubbleText) {
        ioText << "  " ;
      }
      ioText << "CRC: " << numberString << " (error)\n" ;
    }else if (inBubbleText) {
      ioText << "CRC: " << numberString << "\n" ;
    }
    break ;
  case CRC_DEL_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "CRC DEL\n" ;
    }
    break ;
  case ACK_FIELD_RESULT :
    if (inBubbleText) {
      if (inFrame.mData1 != 0) {
        ioText << "NAK\n" ;
      }else{
        ioText << "ACK\n" ;
      }
    }
    break ;
  case ACK_DEL_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "ACK DEL\n" ;
    }
    break ;
  case EOF_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "End of Frame\n" ;
    }
    break ;
  case INTERMISSION_FIELD_RESULT :
    if (inBubbleText) {
      ioText << "3-bit intermission\n" ;
    }else{
      const U64 frameSampleCount = inFrame.mData1 ;
      ioText << "  Length: " << ((frameSampleCount + samplesPerBit / 2) / samplesPerBit) << " bits ("
             << (frameSampleCount * 1000000 / sampleRateHz) << " µs)\n"
             << "  " << inFrame.mData2 << " stuff bit"
             << ((inFrame.mData2 > 1) ? "s" : "")
             << "\n" ;
    }
    break ;
  case CAN_ERROR_RESULT :
    ioText << CanErrorReasonText (CanErrorReason (inFrame.mData1)) << "\n" ;
    break ;
  default :
    ioText << "Error\n" ;
    break ;
  }
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateBubbleText (const U64 inFrameIndex,
                                                     Channel & channel,
                                                     const DisplayBase inDisplayBase) {
  const Frame frame = GetFrame (inFrameIndex) ;
  std::stringstream text ;
  GenerateText (frame, inDisplayBase, true, text) ;
  ClearResultStrings () ;
  AddResultString (text.str().c_str ()) ;
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateFrameTabularText (const U64 inFrameIndex,
                                                           const DisplayBase inDisplayBase) {
  // Per-field Frame objects are kept only to position the waveform bubbles
  // (GenerateBubbleText, above, is untouched and still uses them). The
  // Data Table is fully covered by the one consolidated FrameV2 row per
  // message (emitConsolidatedFrameV2 in CANMolinaroAnalyzer.cpp), so this
  // used to just be redundant clutter -- a generic "Value" column showing
  // per-field text (raw ID, each individual data byte, etc.) alongside
  // the properly named FrameV2 columns that already cover the same data.
  ClearTabularText () ;
}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateExportFile (const char* file,
                                                     DisplayBase display_base,
                                                     U32 export_type_user_id )
{
  std::ofstream file_stream( file, std::ios::out );

  U64 trigger_sample = mAnalyzer->GetTriggerSample();
  U32 sample_rate = mAnalyzer->GetSampleRate();

  file_stream << "Time [s],Value" << std::endl;

  U64 num_frames = GetNumFrames();
  for( U32 i=0; i < num_frames; i++ )
  {
    Frame frame = GetFrame( i );

    char time_str[128];
    AnalyzerHelpers::GetTimeString( frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128 );

    char number_str[128];
    AnalyzerHelpers::GetNumberString( frame.mData1, display_base, 8, number_str, 128 );

    file_stream << time_str << "," << number_str << std::endl;

    if( UpdateExportProgressAndCheckForCancel( i, num_frames ) == true )
    {
      file_stream.close();
      return;
    }
  }

  file_stream.close();
}

//----------------------------------------------------------------------------------------


void CANMolinaroAnalyzerResults::GeneratePacketTabularText( U64 packet_id, DisplayBase display_base )
{
  //not supported

}

//----------------------------------------------------------------------------------------

void CANMolinaroAnalyzerResults::GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base )
{
  //not supported
}

//----------------------------------------------------------------------------------------
