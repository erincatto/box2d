@
@ Copyright (c) 2014 Google, Inc.
@
@ This software is provided 'as-is', without any express or implied
@ warranty.  In no event will the authors be held liable for any damages
@ arising from the use of this software.
@ Permission is granted to anyone to use this software for any purpose,
@ including commercial applications, and to alter it and redistribute it
@ freely, subject to the following restrictions:
@ 1. The origin of this software must not be misrepresented; you must not
@ claim that you wrote the original software. If you use this software
@ in a product, an acknowledgment in the product documentation would be
@ appreciated but is not required.
@ 2. Altered source versions must be plainly marked as such, and must not be
@ misrepresented as being the original software.
@ 3. This notice may not be removed or altered from any source distribution.
@
        .text
        .syntax   unified

        .balign   4
        .global   CalculateTags_Simd
        .thumb_func

CalculateTags_Simd:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ int CalculateTags_Simd(const b2Vec2* positions,
        @                        int count,
        @                        const float& inverseDiameter,
        @                        uint32* outTags)
        @
        @  r0: *positions
        @  r1: count
        @  r2: &inverseDiameter
        @  r3: *outTags
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        @  q0 == x
        @  q1 == y
        @  q2 ==
        @  q3 ==
        @  q4 ==
        @  q5 ==
        @  q6 ==
        @  q7 ==
        @  q8 ==
        @  q9 ==
        @ q10 ==
        @ q11 ==
        @ q12 == inverseDiameter
        @ q13 == xScale
        @ q14 == xOffset
        @ q15 == yOffset

        @ Load constants. Literals are > 32, so must load as integers first.
        vld1.f32          {d24[],d25[]}, [r2] @ q12 = inverseDiameter
        vmov.i32          q13, #0x100   @ q13 = xScale = 1 << 8
        vmov.i32          q14, #0x80000 @ q14 = xOffset = (1 << 8) * (1 << 11)
                                        @               = (1 << 19) = 524288
        vmov.i32          q15, #0x800   @ q15 = xScale = 1 << 11 = 2048
        vcvt.f32.u32      q13, q13      @ convert to float
        vcvt.f32.u32      q14, q14
        vcvt.f32.u32      q15, q15

        @ Calculate tags four at a time, from positions.
.L_CalculateTags_MainLoop:
        @ We consume 32-bytes per iteration, so prefetch 4 iterations ahead.
        @ TODO: experiment with different prefetch lengths on different
        @ architectures.
        pld               [r0, #128]    @ Prefetch position data

        @ {q0, q1} == xPosition and yPosition
        @ Four values in each. q0 = (x0, x1, x2, x3)
        vld2.f32          {q0, q1}, [r0]! @ Read in positions; increment ptr

        @ Calculate tags four at a time.
        vmul.f32          q0, q0, q12   @ q0 = x = xPosition * inverseDiameter
        vmul.f32          q1, q1, q12   @ q1 = y = yPosition * inverseDiameter
        vmul.f32          q0, q0, q13   @ q0 = x * xScale
        vadd.f32          q1, q1, q15   @ q1 = y + yOffset
        vadd.f32          q0, q0, q14   @ q0 = x * xScale + xOffset
        vcvt.u32.f32      q1, q1        @ q1 = (uint32)(y + yOffset)
        vcvt.u32.f32      q0, q0        @ q0 = (uint32)(x * xScale + xOffset)
        vsli.u32          q0, q1, #20   @ q0 = tag
                                        @    = ((uint32)(y + yOffset) <<yShift)
                                        @    + (uint32)(xScale * x + xOffset)

        @ Decrement loop counter; sets the 'gt' flag used in 'bgt' below.
        @ Pipelining is best if there are instructions between the 'subs' and
        @ 'bgt' instructions, since it takes a few cycles for the result of
        @ 'subs' to propegate to the flags register.
        subs              r1, r1, #4

        @ Write out, ignoring index.
        pld               [r3, #64]     @ Prefetch output tag array
        vst1.f32          {q0}, [r3]!   @ write out tags; increment ptr

        bgt               .L_CalculateTags_MainLoop

.L_CalculateTags_Return:
        bx                lr



        .balign   4
        .thumb_func
        @
        @ Once four contacts have been found, calculate their weights and
        @ normals (using SIMD, so all at once).
        @
        @ Also, grab their flags from the flags buffer, and OR them together.
        @ This flag grabbing is slow because we access the flag buffer in a
        @ random order. We use prefetch instructions 'pld' to minimize the
        @ cost of cache misses.
        @
FindContacts_PostProcess:
        @ Preload first four flag addresses into cache.
        @ Note: hardware only has four preload slots.
        ldrh              r9, [r4]
        ldrh              r10, [r4, #2]
        ldrh              r11, [r4, #16]
        ldrh              r12, [r4, #18]
        pld               [r7, r9, lsl #2]
        pld               [r7, r10, lsl #2]
        pld               [r7, r11, lsl #2]
        pld               [r7, r12, lsl #2]

        @ q0 = packedIndices -- indices output to b2ParticleContact
        @ q1 = distBtParticlesSq -- will be used to calculate weight
        @ q2 = diffX -- will be used to calculate normal
        @ q3 = diffY -- will be used to calculate normal
        add               r8, r4, #32
        vld4.f32          {d0, d2, d4, d6}, [r4]
        vld4.f32          {d1, d3, d5, d7}, [r8]

        @ Use distSq to estimate 1 / dist.
        vrsqrte.f32       q8, q1      @ q8 = 1 / dist -- (rough estimate)
        vmul.f32          q9, q8, q1  @ q9 = 1 / dist * distSq -- (appr 'dist')
        vrsqrts.f32       q9, q9, q8  @ q9 = (3 - 1/dist * dist) / 2 -- (error)
        vmul.f32          q8, q8, q9  @ q8 = (error) / dist -- (estimate)
        vcgt.f32          q9, q8, #0  @ q8 = 1 / dist > 0 (true if not NaN)
        vand              q8, q8, q9  @ q8 = 1 / dist if valid, or 0 if NaN

        @ Since we expand the output to include 'weight', we need to preserve
        @ subsequent contacts. Note that there may be up to 7 contacts waiting
        @ to be post-processed, since we output contacts in up-to groups of 4.
        add               r8, r4, #64
        vldmia            r8, {q9, q10, q11}

        @ Load first four flags, 'or' them in pairs, then write to destination.
        ldr               r9, [r7, r9, lsl #2]
        ldr               r10, [r7, r10, lsl #2]
        ldr               r11, [r7, r11, lsl #2]
        ldr               r12, [r7, r12, lsl #2]
        orr               r9, r9, r10
        orr               r11, r11, r12
        str               r9, [r4, #16]
        str               r11, [r4, #36]

        @ Preload the next four flags into cache.
        ldrh              r9, [r4, #32]
        ldrh              r10, [r4, #34]
        ldrh              r11, [r4, #48]
        ldrh              r12, [r4, #50]
        pld               [r7, r9, lsl #2]
        pld               [r7, r10, lsl #2]
        pld               [r7, r11, lsl #2]
        pld               [r7, r12, lsl #2]

        @ Calculate normal and weight.
        vmul.f32          q1, q1, q8     @ q1 = distSq / dist = dist
        vmul.f32          q2, q2, q8     @ q2 = normX = diffX / dist
        vmul.f32          q1, q1, q14    @ q1 = dist / diameter
        vmul.f32          q3, q3, q8     @ q3 = normY = diffY / dist
        vsub.f32          q1, q12, q1    @ q1 = weight = 1 - dist / diameter

        @ Store again, making room for 'weight' member variable this time.
        @ TODO OPT: Interleave with 'or' instructions below.
        mov               r8, #20        @ r8 = 20 = sizeof(b2ParticleContact)
        vst4.f32          {d0[0], d2[0], d4[0], d6[0]}, [r4], r8
        vst4.f32          {d0[1], d2[1], d4[1], d6[1]}, [r4], r8
        vst4.f32          {d1[0], d3[0], d5[0], d7[0]}, [r4], r8
        vst4.f32          {d1[1], d3[1], d5[1], d7[1]}, [r4], r8
        mov               r8, #12        @ r8 = 12 = sizeof(FindContactInput)

        @ Load next four flags, 'or' them in pairs, then write to destination.
        ldr               r9, [r7, r9, lsl #2]
        ldr               r10, [r7, r10, lsl #2]
        ldr               r11, [r7, r11, lsl #2]
        ldr               r12, [r7, r12, lsl #2]
        orr               r9, r9, r10
        orr               r11, r11, r12
        str               r9, [r4, #-24]
        str               r11, [r4, #-4]

        @ Update output pointers. Since we output 4 contacts, and added 4 bytes
        @ for 'weight' on each contact, the output pointer must be advanced by
        @ 16 bytes.
        add               r3, r3, #16
        add               r5, r5, #4          @ numContacts += 4

        @ Restore subsequent contacts. That is, contacts that have yet to be
        @ post-processed.
        vstmia            r4, {q9, q10, q11}

        bx                lr


        @ When used with the 'vtbl' instruction, grabs the first byte of every
        @ word, and places it in the first word. Fills the second word with 0s.
        @ For example, (0xFFFFFFFF, 0x00000000, 0x00000000, 0xFFFFFFFF)
        @              ==> (0xFF0000FF, 0x00000000)
CONST_IS_CLOSE_TABLE_INDICES:
        .byte           0
        .byte           4
        .byte           8
        .byte           12
        .byte           0xFF
        .byte           0xFF
        .byte           0xFF
        .byte           0xFF


        .balign   4
        .global   FindContactsFromChecks_Simd
        .thumb_func
FindContactsFromChecks_Simd:
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @ void FindContactsFromChecks_Simd(
        @       const FindContactInput* reordered,
        @       const FindContactCheck* checks,
        @       int numChecks,
        @       const float& particleDiameterSq,
        @       const float& particleDiameterInv,
        @       const uint32* flags,
        @       b2GrowableBuffer<b2ParticleContact>& contacts)
        @
        @    Parameters
        @  r0: *reordered
        @  r1: *checks
        @  r2: numChecks
        @  r3: particleDiameterSq
        @  [sp]: particleDiameterInv
        @  [sp+4]: *flags
        @  [sp+8]: contacts
        @
        @    Persistent Variables
        @  r0: *reordered (constant)
        @  r1: *checks (advance once per iteration)
        @  r2: numChecks (decrement once per iteration)
        @  r3: *out <-- next free entry of outContacts array
        @  r4: *postProcess <-- entry on-deck to be post-processed
        @  r5: numContacts
        @  r6: maxSafeContacts
        @  r7: *flags (constant)
        @  r8: 20 = sizeof(b2ParticleContact), or
        @      12 = sizeof(FindContactInput) (constants)
        @
        @    Scratch Variables
        @  r9:
        @  r10: address of current particle position
        @  r11: address of comparator particle positions
        @  r12: isClose (compacted)
        @
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        @
        @    Scratch
        @  q0 == index     ------> packedIndices
        @  q1 == positionX ---_    distBtParticlesSq
        @  q2 == positionY --_ --> normX
        @  q3 ==              ---> normY
        @
        @    Unused (note: these are callee-saved)
        @  q4 ==
        @  q5 ==
        @  q6 ==
        @  q7 ==
        @
        @    Scratch
        @  q8 == comparatorIndices
        @  q9 == comparatorPositionX
        @ q10 == comparatorPositionY
        @ q11 ==
        @
        @    Constants
        @ q12 == 1.0f
        @ q13 == isClose table indices
        @ q14 == 1 / particleDiameter
        @ q15 == particleDiameterSq

        push              {r4-r11, lr}

        @ Load constants from registers and stack.
        vld1.f32          {d30[],d31[]}, [r3]  @ q15 = particleDiameterSq
        ldr               r12, [sp, #36]       @ r12 = particleDiameterInv
        vld1.f32          {d28[],d29[]}, [r12] @ q14 = particleDiameterInv
        ldr               r9, [sp, #44]    @ r9 = contacts
        ldr               r7, [sp, #40]    @ r7 = flags
        ldr               r3, [r9, #0]     @ r3 = out = contacts.data
        ldr               r6, [r9, #8]     @ r6 = contacts.capacity
        mov               r4, r3           @ r4 = postProcess = outContacts
        mov               r5, #0           @ r5 = numContacts
        sub               r6, r6, #8       @ r6 = maxSafeContacts = capacity - 8
        mov               r8, #12          @ r8 = 12 = sizeof(FindContactInput)

        @ Perform zero iterations if 'numChecks' is empty.
        @ Must happen after initializing r5 = numContacts = 0.
        cmp               r2, #0
        ble               .L_FindContacts_Return

        @ Load and calculate remaining constants.
        vmov.f32          q12, #1.0        @ q12 = 1.0f splatted
        adr               r12, CONST_IS_CLOSE_TABLE_INDICES
        vld1.8            {d26}, [r12]     @ q13 = *CONST_IS_CLOSE_TABLE_INDICES

.L_FindContacts_MainLoop:
        pld               [r1, #8]         @ prefetch two loops ahead

        @ r10 <== Address of 'position', the current particle position
        @ r11 <== Address of '&comparator[0]', the first particle position we
        @         compare against.
        ldr               r10, [r1], #4    @ r10 = positionIndex|comparatorIndex
        smlatb            r11, r10, r8, r0 @ r11 = address of first comparator
        smlabb            r10, r10, r8, r0 @ r10 = address of current input
        add               r12, r11, #24    @ r12 = address of third comparator

        @ Exit if not enough space in output array (part 1)
        cmp               r5, r6

        @ {q0, q1, q2} == index, positionX, positionY, splatted across vector
        vld3.f32          {d0[], d2[], d4[]}, [r10]
        vld3.f32          {d1[], d3[], d5[]}, [r10]

        @ {q8, q9, q10} == comparatorIndices, comparatorPosX and comparatorPosY
        @                 positions we compare against (positionX, positionY)
        vld3.f32          {d16, d18, d20}, [r11]
        vld3.f32          {d17, d19, d21}, [r12]

        @ q0 = packedIndices -- indices output to b2ParticleContact
        @ q1 = distBtParticlesSq -- will be used to calculate weight
        @ q2 = diffX -- will be used to calculate normal
        @ q3 = diffY -- will be used to calculate normal
        vsub.f32          q3, q10, q2  @ q3 = diffY = comparatorPosY - positionY
        vsub.f32          q2, q9, q1   @ q2 = diffX = comparatorPosX - positionX
        vsli.32           q0, q8, #16  @ q0 = comparatorIndex[i] << 16 | index
        vmul.f32          q1, q3, q3   @ q1 = diffX * diffX
        vmla.f32          q1, q2, q2   @ q1 = diffX * diffX + diffY * diffY

        @ Determine if each particle is close enough to output.
        @ Pack the isClose bitmap (four T or F) into a 32-bit bitmap.
        @ Move 32-bit bitmap to CPU register, for conditional operations.
        @ Note: NEON to CPU register moves are slow (20 cyclds) on some
        @ implementations of NEON.
        @   isClose = distBtParticlesSq < particleDiameterSq
        vclt.f32          q8, q1, q15          @ q8 == isClose
        vtbl.8            d16, {d16,d17}, d26  @ q8[0] == isClose(packed)
        vmov.32           r12, d16[0]          @ q8[0] ==> r12.

        @ If not enough space in output array, grow it.
        @ This is a heavy operation, but should happen rarely.
        ble               .L_FindContacts_Output
        ldr               r9, [sp, #44]        @ r9 = contacts
        str               r5, [r9, #4]         @ contacts.count = numContacts
        ldr               r10, [r9, #0]        @ r10 = contacts.data
        push              {r0-r3, r9, r10, r12}
        vpush             {q0, q1, q2, q3}
        vpush             {q12, q13, q14, q15}
        mov               r0, r9               @ r0 = contacts
        bl                GrowParticleContactBuffer
        vpop              {q12, q13, q14, q15}
        vpop              {q0, q1, q2, q3}
        pop               {r0-r3, r9, r10, r12}

        @ The output array was reallocated, so update 'out', 'postProcess' and
        @ 'maxSafeContacts' pointers.
        ldr               r6, [r9, #8]         @ r6 = contacts.capacity
        ldr               r9, [r9, #0]         @ r9 = contacts.data
        sub               r9, r9, r10          @ r9 = data buffer offset
        sub               r6, r6, #8           @ r6 = maxSafeContacts
        add               r3, r3, r9           @ r3 += data buffer offset
        add               r4, r4, r9           @ r4 += data buffer offset

.L_FindContacts_Output:
        @ Store results to memory, but only results that are close
        tst               r12, 0xFF
        it                ne
        vst4ne.32         {d0[0],d2[0],d4[0],d6[0]}, [r3]! @ Store 1st contact

        tst               r12, 0xFF00
        it                ne
        vst4ne.32         {d0[1],d2[1],d4[1],d6[1]}, [r3]! @ Store 2nd contact

        tst               r12, 0xFF0000
        it                ne
        vst4ne.32         {d1[0],d3[0],d5[0],d7[0]}, [r3]! @ Store 3rd contact

        tst               r12, 0xFF000000
        it                ne
        vst4ne.32         {d1[1],d3[1],d5[1],d7[1]}, [r3]! @ Store 4th contact

        @ post-process the last four elements that have been output
        @ r12 = 5th element to not be post-processed yet
        add               r12, r4, #64         @ r12 = nextPostProcess
        cmp               r3, r12
        it                ge
        blge              FindContacts_PostProcess

        @ decrement loop counter; sets the 'gt' flag used in 'bgt' below
        subs              r2, r2, #1
        bgt               .L_FindContacts_MainLoop

.L_FindContacts_PostProcessRemainingItems:
        @ If at least one output item needs post-processing, do it.
        subs              r12, r3, r4
        ble               .L_FindContacts_Return

        @ r12/16 = num extra contacts to process
        add               r5, r5, r12, lsr #4  @ numContacts += num extra
        push              {r5}                 @ Save numContacts, since stomped

        @ Ensure indices past end of array are zeroed out.
        @ We process 4 contacts in FindContacts_PostProcess, even if we only
        @ have one left to process.
        mov               r12, #0
        str               r12, [r3]
        str               r12, [r3, #16]
        str               r12, [r3, #32]

        bl                FindContacts_PostProcess
        pop               {r5}                 @ Restore numContacts

.L_FindContacts_Return:
        @ Set the final number of contacts in the output buffer.
        ldr               r9, [sp, #44]        @ r9 = contacts
        str               r5, [r9, #4]         @ contacts.count = numContacts

        @ Return by popping the original lr into pc.
        pop               {r4-r11, pc}
