// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

/** Add your docs here. */
public class Node {
    public Integer level;
    public Integer gamePiece;

    public Node(int level, int gamePiece){
        this.level = level;
        this.gamePiece = gamePiece;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result
                + ((level == null) ? 0 : level.hashCode());
        return result;
    }

    @Override
    public boolean equals(final Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
            
        final Node other = (Node) obj;
        if (level == null) {
            if (other.level != null)
                return false;
        } else if (!level.equals(other.level))
            return false;
        if (gamePiece == null) {
            if (other.gamePiece != null)
                return false;
        } else if (!gamePiece.equals(other.gamePiece))
                return false;            
        return true;
    }
}
