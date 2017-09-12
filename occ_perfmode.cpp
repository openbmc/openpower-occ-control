/**
 * Copyright © 2017 IBM Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "occ_perfmode.hpp"
namespace open_power
{
namespace occ
{
namespace performance
{

// For now, just use the Base version. This function is
// added to enable any future commits needing to do extra
// things
bool Mode::mode(bool value)
{
    return Base::Mode::mode(value);
}

} // namespace performance
} // namespace occ
} // namespace open_power
