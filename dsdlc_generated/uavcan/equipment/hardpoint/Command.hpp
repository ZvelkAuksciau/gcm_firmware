/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/valaitis/workspace/kmti/gimbal_ctrl/modules/libuavcan/dsdl/uavcan/equipment/hardpoint/1070.Command.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Generic cargo holder/hardpoint command.
#

uint8 hardpoint_id

#
# Either a binary command (0 - release, 1+ - hold) or bitmask
#
uint16 command
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.hardpoint.Command
saturated uint8 hardpoint_id
saturated uint16 command
******************************************************************************/

#undef hardpoint_id
#undef command

namespace uavcan
{
namespace equipment
{
namespace hardpoint
{

template <int _tmpl>
struct UAVCAN_EXPORT Command_
{
    typedef const Command_<_tmpl>& ParameterType;
    typedef Command_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > hardpoint_id;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > command;
    };

    enum
    {
        MinBitLen
            = FieldTypes::hardpoint_id::MinBitLen
            + FieldTypes::command::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::hardpoint_id::MaxBitLen
            + FieldTypes::command::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::hardpoint_id >::Type hardpoint_id;
    typename ::uavcan::StorageType< typename FieldTypes::command >::Type command;

    Command_()
        : hardpoint_id()
        , command()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<24 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1070 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.hardpoint.Command";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool Command_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        hardpoint_id == rhs.hardpoint_id &&
        command == rhs.command;
}

template <int _tmpl>
bool Command_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(hardpoint_id, rhs.hardpoint_id) &&
        ::uavcan::areClose(command, rhs.command);
}

template <int _tmpl>
int Command_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::hardpoint_id::encode(self.hardpoint_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::command::encode(self.command, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Command_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::hardpoint_id::decode(self.hardpoint_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::command::decode(self.command, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Command_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xA1A036268B0C3455ULL);

    FieldTypes::hardpoint_id::extendDataTypeSignature(signature);
    FieldTypes::command::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Command_<0> Command;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::hardpoint::Command > _uavcan_gdtr_registrator_Command;

}

} // Namespace hardpoint
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::hardpoint::Command >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::hardpoint::Command::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::hardpoint::Command >::stream(Stream& s, ::uavcan::equipment::hardpoint::Command::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "hardpoint_id: ";
    YamlStreamer< ::uavcan::equipment::hardpoint::Command::FieldTypes::hardpoint_id >::stream(s, obj.hardpoint_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "command: ";
    YamlStreamer< ::uavcan::equipment::hardpoint::Command::FieldTypes::command >::stream(s, obj.command, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace hardpoint
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::hardpoint::Command::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::hardpoint::Command >::stream(s, obj, 0);
    return s;
}

} // Namespace hardpoint
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_HPP_INCLUDED