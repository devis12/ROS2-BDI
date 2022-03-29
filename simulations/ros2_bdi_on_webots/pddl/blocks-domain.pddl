;; domain file: blocks-domain.pddl

(define (domain blocks-domain)

    (:requirements :strips :typing :fluents :durative-actions :negative-preconditions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        box stackbase - box_or_stackbase
        gripper
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    (:predicates
        ( upon ?g - gripper ?s - stackbase)
        ( on ?b1 - box ?b2 - box_or_stackbase ?base - stackbase )
        ( in ?sb1 - box_or_stackbase ?sb2 - box_or_stackbase )
        ( clear ?b - box_or_stackbase )
        ( holding ?g - gripper ?b - box )
    )

    (:functions
        (moving_boxes ?g - gripper)
    )

    (:durative-action gripper_pickup
        :parameters (?g - gripper ?b1 - box ?bs2 - box_or_stackbase ?base - stackbase)
        :duration (= ?duration 3)
        :condition (and
            (at start (on ?b1 ?bs2 ?base))
            (at start (in ?b1 ?base))
            (at start (in ?bs2 ?base))
            (at start (clear ?b1))
            (at start (< (moving_boxes ?g) 1))
            (over all (upon ?g ?base))
        )
        :effect (and
            (at start (increase (moving_boxes ?g) 1))
            (at end (not(in ?b1 ?base)))
            (at end (holding ?g ?b1))
            (at end (not(clear ?b1)))
            (at end (not(on ?b1 ?bs2 ?base)))
            (at end (clear ?bs2))
        )
    )

    (:durative-action gripper_putdown
        :parameters (?g - gripper ?b1 - box ?bs2 - box_or_stackbase ?base - stackbase)
        :duration (= ?duration 3)
        :condition (and
            (at start (holding ?g ?b1))
            (at start (clear ?bs2))
            (over all (upon ?g ?base))
            (at start (in ?bs2 ?base))
        )
        :effect (and
            (at start (not(clear ?bs2)))
            (at end (decrease (moving_boxes ?g) 1))
            (at end (not (holding ?g ?b1)))
            (at end (clear ?b1))
            (at end (on ?b1 ?bs2 ?base))
            (at end (in ?b1 ?base))
        )
    )

    (:durative-action move_gripper
        :parameters (?g - gripper ?sb1 ?sb2 - stackbase)
        :duration (= ?duration 4)
        :condition (and
            (at start (upon ?g ?sb1))
        )
        :effect (and
            (at start (not(upon ?g ?sb1)))
            (at end (upon ?g ?sb2))
        )
    )
)
