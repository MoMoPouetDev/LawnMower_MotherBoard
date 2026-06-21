#ifndef _FSM_ENUM_H_
#define _FSM_ENUM_H_

#define PHASE_INIT_INIT                                             0x00
#define PHASE_DOCK_INIT                                             0x01
#define PHASE_DOCK_WAITING_FOR_LEAVING_DOCK                         0x0F   //15

#define PHASE_OPERATIVE_INIT                                        0x10   //16
#define PHASE_OPERATIVE_WAITING_FOR_RETURN_TO_BASE                  0x1F   //31

#define PHASE_RETURN_TO_BASE_INIT                                   0x20   //32
#define PHASE_RETURN_TO_BASE_WAITING_DOCKING                        0x2F   //47

#define PHASE_ERROR_INIT                                            0x30   //46
#define PHASE_ERROR_WAITING_FOR_USER_DECISION                       0x3F   //61

/* DECLARATION DES ETATS DU MODULE SUPERVISOR */
 typedef enum
 {
     S_SUP_INIT_Init                                = PHASE_INIT_INIT,

     S_SUP_DOCK_Init                                = PHASE_DOCK_INIT,
     S_SUP_DOCK_In_Charge                           = S_SUP_DOCK_Init + 1U,
     S_SUP_DOCK_Waiting_For_Mow                     = S_SUP_DOCK_In_Charge + 1U,
     S_SUP_DOCK_Waiting_For_Leaving_Dock            = PHASE_DOCK_WAITING_FOR_LEAVING_DOCK,

     S_SUP_OPERATIVE_Init                           = PHASE_OPERATIVE_INIT,//64
     S_SUP_OPERATIVE_Moving                         = S_SUP_OPERATIVE_Init + 1U,//68
     S_SUP_OPERATIVE_Wire_Detection                 = S_SUP_OPERATIVE_Moving + 1U,
     S_SUP_OPERATIVE_Bumper_Detection               = S_SUP_OPERATIVE_Wire_Detection + 1U,
     S_SUP_OPERATIVE_Waiting                        = S_SUP_OPERATIVE_Bumper_Detection + 1U,
     S_SUP_OPERATIVE_Waiting_For_Return_To_Base     = PHASE_OPERATIVE_WAITING_FOR_RETURN_TO_BASE,

     S_SUP_RETURN_TO_BASE_Init                      = PHASE_RETURN_TO_BASE_INIT, //80
     S_SUP_RETURN_TO_BASE_Angle_To_Base             = S_SUP_RETURN_TO_BASE_Init + 1U,
     S_SUP_RETURN_TO_BASE_Moving                    = S_SUP_RETURN_TO_BASE_Angle_To_Base + 1U,
     S_SUP_RETURN_TO_BASE_Wire_Detection            = S_SUP_RETURN_TO_BASE_Moving + 1U,//82
     S_SUP_RETURN_TO_BASE_Bumper_Detection          = S_SUP_RETURN_TO_BASE_Wire_Detection + 1U,
     S_SUP_RETURN_TO_BASE_Wire_Guiding              = S_SUP_RETURN_TO_BASE_Bumper_Detection + 1U,//83
     S_SUP_RETURN_TO_BASE_Waiting_For_Docking      = PHASE_RETURN_TO_BASE_WAITING_DOCKING,//95

     S_SUP_ERROR_Init                               = PHASE_ERROR_INIT,
     S_SUP_ERROR_Waiting_For_User_Decision          = PHASE_ERROR_WAITING_FOR_USER_DECISION
 }S_MOWER_FSM_STATE;

void FSM_Enum_SetFsmPhase(S_MOWER_FSM_STATE _FSM_Phase);
S_MOWER_FSM_STATE FSM_Enum_GetFsmPhase(void);

 #endif /* _FSM_ENUM_H_ */
