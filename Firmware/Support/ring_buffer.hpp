#pragma once

#include <stddef.h>
#include <stdint.h>
#include <type_traits>

namespace cms32::support
{

constexpr bool is_power_of_two(size_t value) noexcept
{
    return (value != 0U) && ((value & (value - 1U)) == 0U);
}

// SPSC ring buffer: one ISR producer, one main-loop consumer.
template <typename T, size_t Capacity> class RingBuffer
{
public:
    static_assert(is_power_of_two(Capacity), "Capacity must be power of two");
    static_assert(Capacity >= 2U, "Capacity must leave one empty slot");
    static_assert(std::is_trivially_copyable<T>::value,
                  "RingBuffer item must be trivially copyable");

    RingBuffer() = default;
    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

    bool push_isr(const T& value) noexcept
    {
        const size_t head = head_;
        const size_t next = wrap(head + 1U);
        if (next == tail_)
        {
            overflow_count_++;
            return false;
        }

        data_[head] = value;
        head_ = next;
        return true;
    }

    bool pop(T& out) noexcept
    {
        const size_t tail = tail_;
        if (tail == head_)
        {
            return false;
        }

        out = data_[tail];
        tail_ = wrap(tail + 1U);
        return true;
    }

    bool empty() const noexcept
    {
        return head_ == tail_;
    }

    bool full() const noexcept
    {
        return wrap(head_ + 1U) == tail_;
    }

    size_t size() const noexcept
    {
        return wrap(head_ - tail_);
    }

    static constexpr size_t capacity() noexcept
    {
        return Capacity - 1U;
    }

    uint32_t overflow_count() const noexcept
    {
        return overflow_count_;
    }

    // Only call when the producer cannot write concurrently.
    void clear() noexcept
    {
        head_ = 0U;
        tail_ = 0U;
        overflow_count_ = 0U;
    }

private:
    static constexpr size_t wrap(size_t value) noexcept
    {
        return value & (Capacity - 1U);
    }

    T data_[Capacity]{};
    volatile size_t head_{0U};
    volatile size_t tail_{0U};
    volatile uint32_t overflow_count_{0U};
};

} // namespace cms32::support
