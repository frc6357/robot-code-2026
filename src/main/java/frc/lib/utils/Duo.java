package frc.lib.utils;

public class Duo<A, B> {
    private final A m_first;
    private final B m_second;

    /**
     * Constructs a pair.
     *
     * @param first The first object.
     * @param second The second object.
     */
    public Duo(A first, B second) {
        m_first = first;
        m_second = second;
    }

    /**
     * Returns the first object.
     *
     * @return The first object.
     */
    public A getFirst() {
        return m_first;
    }

    /**
     * Returns the second object.
     *
     * @return The second object.
     */
    public B getSecond() {
        return m_second;
    }

    /**
     * Returns a duo comprised of the two given objects.
     *
     * @param <A> The first object's type.
     * @param <B> The second object's type.
     * @param a The first object.
     * @param b The second object.
     * @return A duo comprised of the two given objects.
     */
    public static <A, B> Duo<A, B> of(A a, B b) {
        return new Duo<>(a, b);
    }
    
}
