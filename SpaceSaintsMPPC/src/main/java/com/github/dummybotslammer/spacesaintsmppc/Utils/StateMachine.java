package com.github.dummybotslammer.spacesaintsmppc.Utils;

public class StateMachine {
    private boolean prevstates[];
    private int sequences[];

    //Empty Constructor
    public StateMachine() {
        prevstates = new boolean[] {};
        sequences = new int[] {};
    }

    //Getters
    public boolean[] getPrevStates() {
        return prevstates;
    }

    public int[] getSequences() {
        return sequences;
    }

    public int getSequence(int index) {
        return sequences[Math.min(index, sequences.length-1)];
    }

    //Setters
    public void setPrevStates(boolean[] prevs) {
        prevstates = prevs;
    }

    public void setSequences(int[] seqs) {
        sequences = seqs;
    }

    public void setSequence(int num, int index) {
        sequences[Math.min(index, sequences.length-1)] = num;
    }

    //Methods
    public boolean highToggle(boolean state, int index) {
        int i = Math.min(index, prevstates.length-1);
        boolean prev = prevstates[i];
        if (state && !prev) { prevstates[i]=true; return true; }
        else { prevstates[i] = state; return false; }
    }

    public int incrementSequence(int index) {
        return sequences[Math.min(index, sequences.length-1)]++;
    }

    public int decrementSequence(int index) {
        return sequences[Math.min(index, sequences.length-1)]--;
    }
}
