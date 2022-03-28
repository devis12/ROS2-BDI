( define (problem blocks-problem1)
 ( :domain blocks-domain )
 ( :objects
 	stack1 stack2 stack3 - stack
 	box1 box2 box3 - box
 	floor1 floor2 floor3 - floor
 	gripperA - gripper
 )
 ( :init
 	( upon gripperA stack2 )
 	( free gripperA )
 	( in box1 stack1 )
 	( in box2 stack2 )
 	( in floor1 stack1 )
 	( in floor2 stack2 )
 	( in floor3 stack3 )
 	( on box1 box2 )
 	( on box2 box3 )
 	( on box3 floor1 )
 	( clear box1 )
    ( clear floor2 )
    ( clear floor3 )
 )
 ( :goal 
    (and (on box1 box2) (on box2 box3) (on box3 floor3))
 )
)
