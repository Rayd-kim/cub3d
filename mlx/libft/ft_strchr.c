/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strchr.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/11/17 14:45:14 by youskim           #+#    #+#             */
/*   Updated: 2021/11/29 21:02:09 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

char	*ft_strchr(const char *str, int c)
{
	int		i;
	char	k;

	i = 0;
	k = (char)c;
	while (str[i] != '\0')
	{
		if (str[i] == k)
			return ((char *)&str[i]);
		i++;
	}
	if (k == 0)
		return ((char *)&str[i]);
	return (NULL);
}
