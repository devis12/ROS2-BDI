( define (problem gripper-problem1)
 ( :domain gripper-domain )
 ( :objects
 	start base1 base2 baseA baseB baseC - stackbase
 	boxA1 boxA2	boxB1 boxB2	boxC1 boxC2 - box
 	gripperA - gripper
 )
 ( :init
 	( upon gripperA start )
 	( = (moving_boxes gripperA) 0)

   ( clear boxA2 )
 	( on boxA2 boxC2 base1)
 	( on boxC2 boxA1 base1)
 	( on boxA1 base1 base1)
   ( in boxA2 base1)
   ( in boxC2 base1)
   ( in boxA1 base1)
   ( in base1 base1)
    
   ( clear boxC1 )
 	( on boxC1 boxB2 base2)
 	( on boxB2 boxB1 base2)
 	( on boxB1 base2 base2)
    ( in boxB1 base2)
    ( in boxB2 base2)
    ( in boxC2 base2)
    ( in base2 base2)

    ( clear baseC )
    ( in baseC baseC )
    ( clear baseB )
    ( in baseB baseB )
    ( clear baseA )
    ( in baseA baseA )
 	
 )
 ( :goal 
    (and (in boxA1 baseA) (in boxA2 baseA) (in boxC1 baseC) (in boxC2 baseC) (in boxB1 baseB) (in boxB2 baseB))
 )
)
