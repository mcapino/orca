package rvolib;

public class MutableFloat {

    private float value;

    public MutableFloat(float value) {
        this.value = value;
    }

    public float getValue() {
        return value;
    }

    public void setValue(float value) {
        this.value = value;
    }

    public String toString() {
        return "" + this.value;
    }

}
