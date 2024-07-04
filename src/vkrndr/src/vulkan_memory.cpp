#include <vulkan_memory.hpp>

#include <vulkan_device.hpp>
#include <vulkan_utility.hpp>

#include <vma_impl.hpp>

vkrndr::mapped_memory vkrndr::map_memory(vulkan_device* device,
    VmaAllocation const& allocation)
{
    void* mapped; // NOLINT
    check_result(vmaMapMemory(device->allocator, allocation, &mapped));
    return {allocation, mapped};
}

void vkrndr::unmap_memory(vulkan_device* device, mapped_memory* memory)
{
    vmaUnmapMemory(device->allocator, memory->allocation);
}
